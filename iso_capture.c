// HD60 S: 呪文再生 + iso(alt2) キャプチャ (libusb-1.0, C)
// node-usb が iso 非対応なので C で実装。Windows と同じ iso 経路で EP0x83 から映像(生YUYV想定)を吸う。
//
// build: gcc -O2 iso_capture.c -o iso_capture $(pkg-config --libs --cflags libusb-1.0)
// run  : sudo ./iso_capture [readSec=6] [alt=2] > /dev/null  (映像は captures/stream-iso.bin へ)
//
// ======================================================================
// TODO (未解決の残作業):
//   - MCU 経由の音声パス確立: 現状 SEP payload 8B から音声を抜いているが、
//     ソースによっては無音のまま。IT6802E/IT66121 の unmute 手順 (HD60S_UNMUTE)
//     と 0x509c MCU コマンドの組み合わせが未確定。
//   - パススルー (HDMI OUT へのループスルー): pt-only モードで一時的に
//     出力するが、IT66121 の AV_MUTE/SW_RST/AFE の恒久的解放シーケンスが
//     ホスト側/MCU側どちらの責務か未特定。keepalive-cycle を回避する形の
//     "投げっぱなし" 経路を探索中。
// ======================================================================
//
// ======================================================================
// SECTION 0: ヘッダ / 定数 / グローバル状態
// ======================================================================
#include <libusb-1.0/libusb.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <sys/mman.h>
#include <sys/resource.h>
#include <sched.h>
#include <errno.h>
#include <alsa/asoundlib.h>
#include <pipewire/pipewire.h>
#include <spa/param/audio/format-utils.h>
#include <spa/pod/builder.h>
#include <pthread.h>

static double now_s(void) {
    struct timeval tv; gettimeofday(&tv, NULL);
    return (double)tv.tv_sec + (double)tv.tv_usec / 1e6;
}
#define MAX_WRITE_BYTES (64LL << 20)  // 出力ファイルは先頭64MBまで(2Gbpsで肥大化防止)

#define VID 0x0fd9              // Elgato USB vendor ID
#define PID 0x0074              // HD60 S product ID
#define EP_STREAM 0x83          // iso IN endpoint (映像+SEP embedded audio)
// マジックナンバーの意味:
//   wValue = 0x5066  → I2C ブリッジ経路 (bank+reg 指定で IT6802E/IT66121 を叩く)
//   wValue = 0x509c  → MCU コマンド経路 (Cypress FX3 内の MCU 相当への 2B コマンド)
//   bRequest = 0xC0/0xC6 → ベンダ固有 (詳細は notes/protocol.md 参照予定)
//   bank 0x9a=IT66121 write, 0x9b=IT66121 read setup, 0x9c=IT6802E write, 0x9d=IT6802E read setup
// SuperSpeed iso: 1 iso packet = 1 service interval = wBytesPerInterval まで。
// alt2 は bMaxBurst15/Mult1 → wBytesPerInterval=32768。1024で読むと大半empty(バグ)。
// Windows実測: 32 iso pkt/URB × 16 in-flight = 16MB/64ms でempty 0.07%。
// Linuxではさらにキュー深度を増やして URB 完了→再サブミット遅延の吸収余裕を確保。
// libuvcのLinuxデフォルト=100本。8-64本試行で empty% と CPU 負荷のバランスを取る。
#define ISO_PACKETS 64        // 1転送あたりiso packet(=interval)数（32→64、Windows 1MB match）
#define ISO_PKTSIZE 32768     // alt2 wBytesPerInterval
#define NUM_TRANSFERS 128     // 同時投入数（64→128、URB starvation 回避 2026-07-11）

static FILE* outf;
static FILE* g_rdlog = NULL;      // 差分観測: IN読み応答ログ
static long long total_bytes = 0;
static long pkt_ok = 0, pkt_empty = 0, pkt_err = 0;
static int keep_running = 1;
static int inflight = 0;

// ======================================================================
// SECTION 1: フレーム同期パーサ (映像 + 埋込音声 SEP)
// ======================================================================
// フレーム同期パーサ (2026-07-09 workflow解析で構造判明)
// 実データ形式: 各ライン=3840B + 末尾4Bマーカー、時々SEP(0xff00ff02+12B)挿入。
// 1フレーム=1080ACT+45BLK=1125ライン。マーカー0xff000080=EOL_ACT(実映像行), 0xff0000ab=EOL_BLK(ブランキング行), 0xff00ff02+12=SEP。
// マーカーを剥がした1920x1080 YUYV(4147200B)を v4l2loopback へ出す。
// ==================================================================
#define FRAME_W 1920
#define FRAME_H 1080
#define LINE_BYTES (FRAME_W * 2)                   // 3840
#define FRAME_BYTES (LINE_BYTES * FRAME_H)         // 4,147,200

// マーカー (little-endian 32bit で読む)
#define MK_EOL_ACT 0x800000ffu   // bytes: ff 00 00 80
#define MK_EOL_BLK 0xab0000ffu   // bytes: ff 00 00 ab
#define MK_SEP     0x02ff00ffu   // bytes: ff 00 ff 02, その後12バイトスキップ

typedef enum { HUNT = 0, LOCKED = 1 } pstate_t;
static pstate_t g_state = HUNT;
static uint8_t g_linebuf[LINE_BYTES];              // 現行ライン組み立て
static int g_lpos = 0;                             // linebufの充填位置
static uint8_t g_framebuf[FRAME_BYTES];            // 完成フレーム
static int g_fline = 0;                            // フレーム内のACT行番号
static int g_blk_run = 0;                          // 連続BLKカウンタ

// 32bitマーカーが行またぎで壊れないように、直前の3バイト+SEP繰越を保持する pending バッファ
static uint8_t g_pend[16];
static int g_pend_n = 0;

static int g_v4l_fd = -1;

// ======================================================================
// SECTION 2: ALSA 音声出力 (snd-aloop 経由)
// ======================================================================
// ALSA snd-aloop 出力
// 🔥 2026-07-11 kusq A/B 試聴で判明: SEP 8B payload = 4 mono int16 LE samples
// (連続 4 サンプル、stereo LRLRではない)。native = 96kHz mono (Switch HDMI)。
// iso packet loss で実際に到着する SEP rate < 期待値 (24000/s) → 実効サンプル
// レート < 96kHz → 96kHz で ALSA へ流すと足りない → underrun / 音が遅延。
// 🔥 根本対策 (2026-07-11): 起動時に SEP marker rate を 2 秒実測 → 96kHz への
// 補間比 upsample_ratio を算出 → ALSA は 96kHz 固定のまま、各 SEP からの
// 4 samples を upsample_ratio 倍に線形補間して feed。結果: 音は本来速度で鳴る。
static snd_pcm_t* g_pcm = NULL;
static unsigned long long g_audio_frames = 0;
static unsigned long long g_audio_underrun = 0;
#define AUDIO_BATCH_FRAMES 960  // 10ms 相当 (96kHz mono)
static int16_t g_audio_buf[AUDIO_BATCH_FRAMES];  // mono
static int g_audio_buf_pos = 0;

static int g_sep_count = 0;
static double g_measure_start_s = 0.0;
static int g_measure_done = 0;
static int g_measure_started = 0;   // 初 SEP 到着で開始
// 96kHz へアップサンプルする比率 (0=固定計算前)
static double g_upsample_ratio = 0.0;
// 補間状態: 前回の終点サンプル (次の補間の起点)
static int16_t g_last_sample = 0;
// 分数遅延累積 (次に何 samples 出すかの端数管理)
static double g_frac_pos = 0.0;

static double now_monotonic_s(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec / 1e9;
}

// PipeWire 版の前方宣言 (定義は後段)
static void audio_pw_open(void);
static void audio_pw_close(void);
static void audio_feed_sep_pw(const uint8_t* payload);
extern int g_use_pw;

static void audio_open(const char* pcm_name) {
    // HD60S_AUDIO_PW=1 で PipeWire ネイティブ実装に分岐
    const char* env_pw = getenv("HD60S_AUDIO_PW");
    if (env_pw && env_pw[0] && env_pw[0] != '0' && env_pw[0] != 'n') {
        g_use_pw = 1;
        audio_pw_open();
        return;
    }
    int err = snd_pcm_open(&g_pcm, pcm_name, SND_PCM_STREAM_PLAYBACK, 0);
    if (err < 0) {
        fprintf(stderr, "[audio] snd_pcm_open(%s) failed: %s\n", pcm_name, snd_strerror(err));
        g_pcm = NULL;
        return;
    }
    err = snd_pcm_set_params(g_pcm,
        SND_PCM_FORMAT_S16_LE,
        SND_PCM_ACCESS_RW_INTERLEAVED,
        1,          // mono
        96000,      // 常に 96kHz (下流の arecord|aplay と一致)
        1,          // soft resample
        200000);    // 200ms latency (余裕を持たせる)
    if (err < 0) {
        fprintf(stderr, "[audio] snd_pcm_set_params failed: %s\n", snd_strerror(err));
        snd_pcm_close(g_pcm); g_pcm = NULL;
        return;
    }
    fprintf(stderr, "[audio] ALSA %s opened (96kHz S16_LE mono), 初 SEP 到着から 2 秒で実 rate 測定...\n", pcm_name);
}

static void audio_flush(void) {
    if (!g_pcm || g_audio_buf_pos == 0) return;
    snd_pcm_sframes_t written = snd_pcm_writei(g_pcm, g_audio_buf, g_audio_buf_pos);
    if (written < 0) {
        g_audio_underrun++;
        int err = snd_pcm_recover(g_pcm, (int)written, 1);
        if (err < 0) {
            fprintf(stderr, "[audio] recover failed: %s\n", snd_strerror(err));
        }
    } else {
        g_audio_frames += written;
    }
    g_audio_buf_pos = 0;
}

// 補間 & 出力書き込み。samples[n] を effective rate × upsample_ratio 相当で
// 96kHz に伸ばす (nearest / linear)。
static void audio_feed_sep(const uint8_t* payload) {
    if (g_use_pw) { audio_feed_sep_pw(payload); return; }
    g_sep_count++;

    // 測定期間: 初 SEP 到着してから 2 秒経過 かつ 500 SEP 以上集まるまで待つ
    // (時間ベースだと iso 初期化で SEP が来てない時期に測定して壊れる)
    if (!g_measure_done) {
        if (!g_measure_started) {
            g_measure_start_s = now_monotonic_s();
            g_measure_started = 1;
        }
        double el = now_monotonic_s() - g_measure_start_s;
        if (el < 2.0 || g_sep_count < 500) return;
        double sep_rate = g_sep_count / el;
        double effective_sample_rate = sep_rate * 4.0;
        if (effective_sample_rate < 4000.0 || effective_sample_rate > 200000.0) {
            effective_sample_rate = 96000.0;
        }
        g_upsample_ratio = 96000.0 / effective_sample_rate;
        fprintf(stderr, "[audio] measured: %.1f SEP/s → %.0f Hz eff → upsample %.3fx to 96kHz\n",
                sep_rate, effective_sample_rate, g_upsample_ratio);
        g_measure_done = 1;
        g_last_sample = 0;
        g_frac_pos = 0.0;
        return;
    }

    if (!g_pcm || g_upsample_ratio <= 0) return;

    // 8B payload → 4 mono int16 samples
    const int16_t* s = (const int16_t*)payload;
    for (int k = 0; k < 4; k++) {
        int16_t cur = s[k];
        // 各入力 sample を upsample_ratio 個の出力 sample に展開
        // linear interpolation: prev (g_last_sample) → cur
        double count = g_upsample_ratio + g_frac_pos;
        int n = (int)count;
        g_frac_pos = count - n;
        for (int i = 1; i <= n; i++) {
            double t = (double)i / (double)(n > 0 ? n : 1);
            double v = g_last_sample + (cur - g_last_sample) * t;
            if (v > 32767.0) v = 32767.0;
            if (v < -32768.0) v = -32768.0;
            if (g_audio_buf_pos >= AUDIO_BATCH_FRAMES) audio_flush();
            g_audio_buf[g_audio_buf_pos++] = (int16_t)v;
        }
        g_last_sample = cur;
    }
    if (g_audio_buf_pos >= AUDIO_BATCH_FRAMES) audio_flush();
}

// ======================================================================
// 🔥 libpipewire ネイティブ実装 (Opus 4.8 提案、2026-07-11)
// 前実装: iso_capture → snd_aloop → arecord|aplay → PipeWire = 4段バッファ
// 新実装: iso_capture → pw_stream (直接) = 1段のみ、~2.6ms/quantum に到達
// 環境変数 HD60S_AUDIO_PW=1 で有効化。ALSA 実装との A/B 比較用にフラグ化。
// ======================================================================
static struct pw_thread_loop* g_pw_loop = NULL;
static struct pw_stream* g_pw_stream = NULL;
// ring buffer (mono int16 samples). Producer: iso_capture (audio_feed_sep 経由).
// Consumer: pipewire process コールバック (別スレッド)。
// 🔥 kusq テストで判明: 683ms は大きすぎて古い音蓄積 → 遅延。
// 4096 samples @ 96kHz = 43ms に削減、さらに満杯で古いのから破棄 (最新優先)。
#define PW_RING_SIZE 4096
// PW スレッドが「読み残し」しないように、目標水位 = 1 quantum 分 (128 samples ~1.3ms).
#define PW_TARGET_FILL 256
static int16_t g_pw_ring[PW_RING_SIZE];
static volatile uint32_t g_pw_ring_head = 0;  // producer write index (iso スレッド)
static volatile uint32_t g_pw_ring_tail = 0;  // consumer read index (pw スレッド)
static pthread_mutex_t g_pw_ring_lock = PTHREAD_MUTEX_INITIALIZER;

// PW process コールバック用の状態
static int16_t g_pw_last_sample = 0;  // underflow 時の連続性維持用

// pw_stream process コールバック: PipeWire スレッドから呼ばれる。ring から
// dequeue して PW バッファに書き込む。
// 🔥 音割れ対策: underflow 時は 0 埋めでなく last_sample を decay させる
static void pw_on_process(void* userdata) {
    (void)userdata;
    struct pw_buffer* b = pw_stream_dequeue_buffer(g_pw_stream);
    if (!b) return;
    struct spa_buffer* buf = b->buffer;
    if (!buf->datas[0].data) { pw_stream_queue_buffer(g_pw_stream, b); return; }

    int16_t* dst = (int16_t*)buf->datas[0].data;
    uint32_t max_frames = buf->datas[0].maxsize / sizeof(int16_t);
    uint32_t want = b->requested;
    if (want == 0 || want > max_frames) want = max_frames;

    pthread_mutex_lock(&g_pw_ring_lock);
    uint32_t head = g_pw_ring_head;
    uint32_t tail = g_pw_ring_tail;
    uint32_t avail = (head - tail) & (PW_RING_SIZE - 1);
    pthread_mutex_unlock(&g_pw_ring_lock);

    uint32_t n = (want < avail) ? want : avail;
    for (uint32_t i = 0; i < n; i++) {
        dst[i] = g_pw_ring[(tail + i) & (PW_RING_SIZE - 1)];
    }
    if (n > 0) g_pw_last_sample = dst[n-1];

    // 🔥 underflow 埋め: 0 でなく last_sample を高速に減衰 (連続性維持でクリック回避)
    int32_t decay = g_pw_last_sample;
    for (uint32_t i = n; i < want; i++) {
        // 指数減衰: 25 samples ごとに 3/4 (~1ms で急速に silence)
        decay = decay * 245 / 256;  // 0.957x per sample
        dst[i] = (int16_t)decay;
    }
    g_pw_last_sample = decay;

    pthread_mutex_lock(&g_pw_ring_lock);
    g_pw_ring_tail = (tail + n) & (PW_RING_SIZE - 1);
    pthread_mutex_unlock(&g_pw_ring_lock);

    buf->datas[0].chunk->offset = 0;
    buf->datas[0].chunk->stride = sizeof(int16_t);
    buf->datas[0].chunk->size = want * sizeof(int16_t);
    pw_stream_queue_buffer(g_pw_stream, b);
}

static const struct pw_stream_events g_pw_events = {
    PW_VERSION_STREAM_EVENTS,
    .process = pw_on_process,
};

// ring バッファに samples 書き込み (iso スレッドから)
// 🔥 低遅延化: 溜まりすぎたら古いのから破棄。目標 fill レベルを維持。
static void pw_ring_push(int16_t sample) {
    pthread_mutex_lock(&g_pw_ring_lock);
    uint32_t fill = (g_pw_ring_head - g_pw_ring_tail) & (PW_RING_SIZE - 1);
    if (fill >= PW_RING_SIZE - 8) {
        // ring full: drop oldest (advance tail by 1)
        g_pw_ring_tail = (g_pw_ring_tail + 1) & (PW_RING_SIZE - 1);
    }
    g_pw_ring[g_pw_ring_head] = sample;
    g_pw_ring_head = (g_pw_ring_head + 1) & (PW_RING_SIZE - 1);
    pthread_mutex_unlock(&g_pw_ring_lock);
}

// 🔥 iso スレッドから: fill を slow correction する。ハードな drop はクリック音を
// 生むので、毎 SEP に少しずつだけ tail を進めて滑らかに追いつく。
static void pw_ring_trim_if_too_full(void) {
    pthread_mutex_lock(&g_pw_ring_lock);
    uint32_t fill = (g_pw_ring_head - g_pw_ring_tail) & (PW_RING_SIZE - 1);
    // target の 4倍超えたら 1 sample だけ捨てる。徐々に減っていく。
    if (fill > PW_TARGET_FILL * 4) {
        g_pw_ring_tail = (g_pw_ring_tail + 1) & (PW_RING_SIZE - 1);
    }
    pthread_mutex_unlock(&g_pw_ring_lock);
}

static void audio_pw_open(void) {
    pw_init(NULL, NULL);
    g_pw_loop = pw_thread_loop_new("hd60s-audio", NULL);
    if (!g_pw_loop) {
        fprintf(stderr, "[audio-pw] pw_thread_loop_new failed\n");
        return;
    }
    pw_thread_loop_lock(g_pw_loop);

    struct pw_properties* props = pw_properties_new(
        PW_KEY_MEDIA_TYPE, "Audio",
        PW_KEY_MEDIA_CATEGORY, "Playback",
        PW_KEY_MEDIA_ROLE, "Game",
        PW_KEY_NODE_NAME, "hd60s-monitor",
        PW_KEY_NODE_DESCRIPTION, "HD60 S Monitor",
        PW_KEY_NODE_LATENCY, "128/48000",   // 128 samples @ 48kHz = 2.6ms
        PW_KEY_NODE_ALWAYS_PROCESS, "true",
        NULL);

    g_pw_stream = pw_stream_new_simple(
        pw_thread_loop_get_loop(g_pw_loop),
        "hd60s-monitor",
        props,
        &g_pw_events,
        NULL);

    if (!g_pw_stream) {
        fprintf(stderr, "[audio-pw] pw_stream_new_simple failed\n");
        pw_thread_loop_unlock(g_pw_loop);
        pw_thread_loop_destroy(g_pw_loop);
        g_pw_loop = NULL;
        return;
    }

    // Audio format: S16 LE mono 96kHz (SEP native)
    uint8_t buffer[1024];
    struct spa_pod_builder pod_builder = SPA_POD_BUILDER_INIT(buffer, sizeof(buffer));
    struct spa_audio_info_raw info = SPA_AUDIO_INFO_RAW_INIT(
        .format = SPA_AUDIO_FORMAT_S16_LE,
        .channels = 1,
        .rate = 96000
    );
    const struct spa_pod* params[1];
    params[0] = spa_format_audio_raw_build(&pod_builder, SPA_PARAM_EnumFormat, &info);

    int r = pw_stream_connect(g_pw_stream,
        PW_DIRECTION_OUTPUT,
        PW_ID_ANY,
        PW_STREAM_FLAG_AUTOCONNECT |
        PW_STREAM_FLAG_MAP_BUFFERS |
        PW_STREAM_FLAG_RT_PROCESS,
        params, 1);
    if (r < 0) {
        fprintf(stderr, "[audio-pw] pw_stream_connect failed: %d\n", r);
        pw_thread_loop_unlock(g_pw_loop);
        return;
    }

    pw_thread_loop_unlock(g_pw_loop);
    pw_thread_loop_start(g_pw_loop);
    fprintf(stderr, "[audio-pw] PipeWire ネイティブ stream 起動 (96kHz S16_LE mono, latency=128/48000=2.6ms)\n");
}

static void audio_pw_close(void) {
    if (!g_pw_loop) return;
    pw_thread_loop_stop(g_pw_loop);
    if (g_pw_stream) {
        pw_stream_destroy(g_pw_stream);
        g_pw_stream = NULL;
    }
    pw_thread_loop_destroy(g_pw_loop);
    g_pw_loop = NULL;
    pw_deinit();
}

// audio_feed_sep の PipeWire 版: upsample した samples を ring に push
static void audio_feed_sep_pw(const uint8_t* payload) {
    g_sep_count++;
    if (!g_measure_done) {
        if (!g_measure_started) {
            g_measure_start_s = now_monotonic_s();
            g_measure_started = 1;
        }
        double el = now_monotonic_s() - g_measure_start_s;
        if (el < 2.0 || g_sep_count < 500) return;
        double sep_rate = g_sep_count / el;
        double effective_sample_rate = sep_rate * 4.0;
        if (effective_sample_rate < 4000.0 || effective_sample_rate > 200000.0) {
            effective_sample_rate = 96000.0;
        }
        g_upsample_ratio = 96000.0 / effective_sample_rate;
        fprintf(stderr, "[audio-pw] measured: %.1f SEP/s → %.0f Hz eff → upsample %.3fx to 96kHz\n",
                sep_rate, effective_sample_rate, g_upsample_ratio);
        g_measure_done = 1;
        g_last_sample = 0;
        g_frac_pos = 0.0;
        return;
    }
    if (!g_pw_stream || g_upsample_ratio <= 0) return;

    const int16_t* s = (const int16_t*)payload;
    for (int k = 0; k < 4; k++) {
        int16_t cur = s[k];
        double count = g_upsample_ratio + g_frac_pos;
        int n = (int)count;
        g_frac_pos = count - n;
        for (int i = 1; i <= n; i++) {
            double t = (double)i / (double)(n > 0 ? n : 1);
            double v = g_last_sample + (cur - g_last_sample) * t;
            if (v > 32767.0) v = 32767.0;
            if (v < -32768.0) v = -32768.0;
            pw_ring_push((int16_t)v);
        }
        g_last_sample = cur;
    }
    // 低遅延化: ring が溜まりすぎたら余剰破棄 (upsample_ratio と PW クロックの微妙な差
    // で徐々に fill が増える → 遅延が延びる) を防ぐ
    pw_ring_trim_if_too_full();
}

// フラグ: PipeWire ネイティブモード (前方宣言済)
int g_use_pw = 0;

static void audio_close(void) {
    if (g_use_pw) {
        audio_pw_close();
        return;
    }
    if (!g_pcm) return;
    audio_flush();
    snd_pcm_drain(g_pcm);
    snd_pcm_close(g_pcm);
    g_pcm = NULL;
}

static unsigned long long g_frames_out = 0;
static unsigned long long g_resyncs = 0;
static unsigned long long g_resync_empty = 0;   // iso empty 起因
static unsigned long long g_resync_marker = 0;  // 未知マーカー起因
static unsigned long long g_resync_overflow = 0;// work overflow 起因

static void parser_reset(const char* why) {
    // 分類は state 問わずカウント（HUNT中の resync も見たい）
    if (strstr(why, "empty")) g_resync_empty++;
    else if (strstr(why, "marker")) g_resync_marker++;
    else if (strstr(why, "overflow")) g_resync_overflow++;
    g_state = HUNT; g_lpos = 0; g_fline = 0; g_blk_run = 0; g_pend_n = 0;
    g_resyncs++;
}

static void emit_frame(void) {
    if (g_v4l_fd >= 0) {
        ssize_t w = write(g_v4l_fd, g_framebuf, FRAME_BYTES);
        if (w != FRAME_BYTES) fprintf(stderr, "[v4l2] short write %zd\n", w);
    }
    g_frames_out++;
    // 60,120,300,600フレームの生YUYVを保存（検証用）
    if (g_frames_out == 60 || g_frames_out == 300 || g_frames_out == 600) {
        char path[256];
        snprintf(path, sizeof(path), "captures/proof/live_frame_%llu.yuv", g_frames_out);
        FILE* fp = fopen(path, "wb");
        if (fp) { fwrite(g_framebuf, 1, FRAME_BYTES, fp); fclose(fp);
                  fprintf(stderr, "[dump] %s\n", path); }
    }
    if ((g_frames_out % 60) == 0) fprintf(stderr, "[emit] %llu frames\n", g_frames_out);
}

// data/len を消費してパーサに突っ込む。iso packet 最大32768B、pending 最大16Bなので
// work は余裕を持って十分大きく取る。
static void parser_feed(const uint8_t* data, size_t len) {
    static uint8_t work[65536 + 64];
    if (g_pend_n + len > sizeof(work)) {
        // 想定外の巨大パケット。切り捨てて resync
        parser_reset("work overflow");
        return;
    }
    memcpy(work, g_pend, g_pend_n);
    memcpy(work + g_pend_n, data, len);
    size_t total = g_pend_n + len;
    g_pend_n = 0;
    size_t i = 0;

    while (i < total) {
        // linebuf に 3840B 詰める（g_lposは前回途中の位置）
        size_t need = LINE_BYTES - g_lpos;
        if (need > 0) {
            size_t avail = total - i;
            size_t take = (avail < need) ? avail : need;
            memcpy(g_linebuf + g_lpos, work + i, take);
            g_lpos += take; i += take;
            if (g_lpos < LINE_BYTES) break;  // ラインが埋まってない=残りは次回に持ち越し
        }

        // マーカーを読むため 4B (SEP なら 16B) 必要。足りなければ pending に退避。
        // ★★★重要★★★ ここで pending に残す時、g_lpos は LINE_BYTES のまま維持する。
        // (次回 parser_feed で g_lpos==LINE_BYTES なので need==0 で埋め直しをスキップ、
        //  そのままマーカー判定に進める)
        if (total - i < 4) {
            size_t r = total - i;
            memcpy(g_pend, work + i, r); g_pend_n = r;
            return;
        }
        uint32_t tag; memcpy(&tag, work + i, 4);

        // SEP: 4B magic + 8B payload + 4B マーカー (EOL_ACT等) = 全16B。
        // 実測: SEP+12 位置に必ず EOL_ACT (ff 00 00 80) が続く（201/201=100%）。
        // → 実装は「SEP magic を見たら +12 進んで直後のマーカー4Bを読み直す」。
        //   SEP payload の 8B は音声データの可能性 (parser では読み飛ばす、音声パーサで別処理)。
        if (tag == MK_SEP) {
            if (total - i < 16) {
                size_t r = total - i;
                memcpy(g_pend, work + i, r); g_pend_n = r;
                return;
            }
            // payload = 8バイト、位置 i+4 .. i+11 (音声: 48kHz s16le stereo, 2フレーム/SEP)
            audio_feed_sep(work + i + 4);
            i += 12;                          // payload 8B までジャンプ (magic 4B + 8B payload)
            memcpy(&tag, work + i, 4);        // マーカー4Bを読み直し
        }
        i += 4;

        if (tag == MK_EOL_ACT) {
            if (g_state != LOCKED) {
                if (g_blk_run >= 30) { g_state = LOCKED; g_fline = 0; }
                else { g_lpos = 0; continue; }
            }
            if (g_blk_run > 0 && g_fline >= FRAME_H) {
                emit_frame();
                g_fline = 0;
            }
            g_blk_run = 0;
            if (g_fline < FRAME_H) memcpy(g_framebuf + g_fline * LINE_BYTES, g_linebuf, LINE_BYTES);
            g_fline++;
            g_lpos = 0;
        } else if (tag == MK_EOL_BLK) {
            g_blk_run++;
            g_lpos = 0;
        } else {
            // 未知マーカー: リセットせず、4B戻して1Bだけ進めて再検索。真のマーカーは3844B以内にあるはず。
            // これで映像バイトが偶発的にマーカーに一致した場合の誤同期を回避。
            i -= 3;
            // linebufのこの位置バイトをずらして「1B早く始まるライン」扱いにするのは複雑なので、
            // LOCKED時はライン破棄扱い(g_lpos=0)して次の真マーカーを探す。
            // 頻度制御: 過度なリセットを避けるためカウントだけ増やす
            g_resync_marker++;
            g_lpos = 0;
        }
    }
}

// URB からデータが完全に飛んだ時 (iso pkt actual_length==0 = empty)。
// iso 0-length pkt が単なる "no data this interval" ならライン位置は保たれるが、
// 実際にはデータ欠落を伴うことが多い→ 進行中ラインを破棄して次のマーカーで再同期。
// ただし LOCKED は維持し frame カウントは崩さない (BLKでラインカウントリセットされる)。
__attribute__((unused))
static void parser_notify_loss(size_t bytes_lost) {
    (void)bytes_lost;
    // 現ライン破棄。次のマーカーが来た時にラインが1本ズレるがフレーム全体は緩やかに回復する。
    g_lpos = 0;
    g_pend_n = 0;
}

// v4l2loopback (/dev/video42) をオープン。フォーマット設定は
// 事前に `v4l2loopback-ctl set-caps /dev/video42 YUYV:1920x1080@60/1` で済ませておく想定。
// (S_FMT を driver から叩くと apt 版0.15.3の挙動でCapture側フォーマットが壊れることがある)
// fps ヒントだけ VIDIOC_S_PARM で 60fps に固定 (OBSがフレームレート一覧に60を出すために必要)
static int v4l2_open(const char* devpath) {
    int fd = open(devpath, O_WRONLY);
    if (fd < 0) { perror("open v4l2loopback"); return -1; }
    // 🔥 2026-07-11 Opus 4.8 診断: write() only では v4l2loopback の timestamp state
    // machine が初期化されず、consumer側でフレーム表示が止まる (VLC "Timestamp conversion
    // failed", ffplay fd=0, mpv hang などの症状)。VIDIOC_S_FMT + STREAMON を明示して
    // OUTPUT ストリームを"公式に開始"する。
    struct v4l2_format vf; memset(&vf, 0, sizeof(vf));
    vf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    vf.fmt.pix.width = 1920;
    vf.fmt.pix.height = 1080;
    vf.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
    vf.fmt.pix.field = V4L2_FIELD_NONE;
    vf.fmt.pix.bytesperline = 3840;
    vf.fmt.pix.sizeimage = 4147200;
    vf.fmt.pix.colorspace = V4L2_COLORSPACE_SRGB;
    if (ioctl(fd, VIDIOC_S_FMT, &vf) < 0) fprintf(stderr, "[v4l2] S_FMT 失敗(続行): %s\n", strerror(errno));

    struct v4l2_streamparm sp; memset(&sp, 0, sizeof(sp));
    sp.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    sp.parm.output.timeperframe.numerator = 1;
    sp.parm.output.timeperframe.denominator = 60;
    if (ioctl(fd, VIDIOC_S_PARM, &sp) < 0) fprintf(stderr, "[v4l2] S_PARM fps60 失敗(続行)\n");

    // STREAMON: OUTPUT ストリーム開始を宣言 (write() モードでも必須)
    enum v4l2_buf_type btype = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    if (ioctl(fd, VIDIOC_STREAMON, &btype) < 0) fprintf(stderr, "[v4l2] STREAMON 失敗(続行): %s\n", strerror(errno));

    fprintf(stderr, "[v4l2] %s opened (YUYV 1920x1080 @60fps, S_FMT+STREAMON 済)\n", devpath);
    return fd;
}

static int hex2bin(const char* hex, unsigned char* out, int maxlen) {
    int n = 0; const char* p = hex;
    while (p[0] && p[1] && n < maxlen) {
        int hi, lo;
        char c = p[0]; hi = (c<='9')?c-'0':(c|32)-'a'+10;
        c = p[1]; lo = (c<='9')?c-'0':(c|32)-'a'+10;
        out[n++] = (hi<<4)|lo; p += 2;
    }
    return n;
}

// ======================================================================
// SECTION 3: USB 送受信 (呪文再生 / iso コールバック / burst 発火)
// ======================================================================
static void LIBUSB_CALL iso_cb(struct libusb_transfer* xfr) {
    // HEX DUMP HOOK: HD60S_HEXDUMP=1 で iso packet の先頭 32B を最初 500 個 dump
    // 音声パケットと映像パケットを見分けるため (workflow suggestion 2026-07-11)
    static const char* env_hexdump = NULL;
    static int hexdump_check = 0;
    static int hexdump_count = 0;
    if (!hexdump_check) {
        env_hexdump = getenv("HD60S_HEXDUMP");
        hexdump_check = 1;
    }
    int do_hexdump = env_hexdump && env_hexdump[0] && env_hexdump[0] != '0';

    for (int i = 0; i < xfr->num_iso_packets; i++) {
        struct libusb_iso_packet_descriptor* d = &xfr->iso_packet_desc[i];
        if (d->status == LIBUSB_TRANSFER_COMPLETED) {
            if (d->actual_length > 0) {
                unsigned char* buf = libusb_get_iso_packet_buffer_simple(xfr, i);
                // HEX DUMP hook - first 500 non-empty packets
                if (do_hexdump && hexdump_count < 500) {
                    fprintf(stderr, "PKT[%d] len=%d: ", hexdump_count, d->actual_length);
                    for (int b = 0; b < 32 && b < d->actual_length; b++) fprintf(stderr, "%02x", buf[b]);
                    fprintf(stderr, "\n");
                    hexdump_count++;
                }
                // parser にライブ供給 (v4l2loopback へ流す)
                parser_feed(buf, d->actual_length);
                // 検証用: 先頭512MBだけ生ストリームを保存 (音声解析用に増量)
                if (total_bytes < (512LL << 20) && outf) fwrite(buf, 1, d->actual_length, outf);
                total_bytes += d->actual_length;
                pkt_ok++;
            } else {
                // iso 0-length pkt はブランキングによる正常な休止と考え、
                // 進行中のライン位置には触れない (実測で empty時にg_lpos リセットすると
                // 逆に marker resync が増える → 触らないのが正解)。
                pkt_empty++;
            }
        } else pkt_err++;
    }
    if (keep_running) {
        if (libusb_submit_transfer(xfr) < 0) { inflight--; }
    } else {
        inflight--;
    }
}

// 呪文再生(TSV) ; 戻り値: (ok<<0) 実際はグローバルでカウント
static void replay_spell(libusb_device_handle* h, const char* path) {
    FILE* f = fopen(path, "r");
    if (!f) { fprintf(stderr, "TSV開けない: %s\n", path); return; }
    char line[8192];
    int ok=0, fail=0, first=1;
    unsigned char data[4096];
    double prev_t = -1;
    int n5066 = 0; char last5066[64] = "";
    int rd_idx = 0; char prev_out[64] = "";   // 差分観測: 直前OUTペイロード
    while (fgets(line, sizeof(line), f)) {
        if (first) { first=0; continue; }              // header
        char c_frame[32], c_time[32], c_brt[16], c_br[16], c_wv[16], c_wi[16], c_wl[16], c_data[8000];
        c_data[0]=0;
        int nf = sscanf(line, "%31[^\t]\t%31[^\t]\t%15[^\t]\t%15[^\t]\t%15[^\t]\t%15[^\t]\t%15[^\t]\t%7999[^\t\n]",
                        c_frame, c_time, c_brt, c_br, c_wv, c_wi, c_wl, c_data);
        if (nf < 7) continue;
        // 元キャプチャのタイミングを再現(コマンド間をsleep)。HDMIロック待ちの150ms間隔もここで再現される。
        double t = atof(c_time);
        if (prev_t >= 0) {
            double dt = t - prev_t;
            if (dt > 0 && dt < 2.0) usleep((useconds_t)(dt * 1e6));   // 上限2s
        }
        prev_t = t;
        unsigned char brt = (unsigned char)strtol(c_brt, NULL, 16);
        unsigned char br  = (unsigned char)strtol(c_br, NULL, 10);
        unsigned short wv = (unsigned short)strtol(c_wv, NULL, 0);
        unsigned short wi = (unsigned short)strtol(c_wi, NULL, 0);
        unsigned short wl = (unsigned short)strtol(c_wl, NULL, 10);
        int is_out = (brt & 0x80) == 0;
        int r;
        if (is_out) {
            int dlen = (nf>=8 && c_data[0]) ? hex2bin(c_data, data, sizeof(data)) : 0;
            r = libusb_control_transfer(h, brt, br, wv, wi, data, dlen, 1000);
            // 差分観測: このOUT(=I2Cセットアップ)を記録。次のIN読みと紐付ける。
            if (nf>=8 && c_data[0]) { strncpy(prev_out, c_data, sizeof(prev_out)-1); prev_out[sizeof(prev_out)-1]=0; }
        } else {
            r = libusb_control_transfer(h, brt, br, wv, wi, data, wl, 1000);
            // 差分観測ログ: 全IN読みを「位置idx / wValue / 直前OUT(I2C対象) / 応答」で記録。
            if (g_rdlog) {
                char rh[64]; int p=0;
                for (int j=0;j<r && j<16;j++) p+=sprintf(rh+p,"%02x",data[j]);
                if (r<=0) { rh[0]='-'; rh[1]=0; }
                fprintf(g_rdlog, "%d\t0x%04x\t%s\t%s\n", rd_idx, wv, prev_out[0]?prev_out:"-", rh);
            }
            rd_idx++;
            // 診断: ステータスポーリング読み(wV=0x5066)の応答を記録し、変化(=ロック検出)を見る
            if (r > 0 && wv == 0x5066 && n5066 < 4000) {
                char hex[64]; int p=0;
                for (int j=0;j<r && j<16;j++) p+=sprintf(hex+p,"%02x",data[j]);
                // 直近と違う応答だけ表示(変化点を捉える)
                if (strcmp(hex, last5066) != 0) {
                    fprintf(stderr, "  [poll@%.2fs] wV5066 resp: %s\n", t, hex);
                    strncpy(last5066, hex, sizeof(last5066)-1);
                }
                n5066++;
            }
        }
        if (r < 0) fail++; else ok++;
    }
    fclose(f);
    fprintf(stderr, "[replay] 呪文再生: ok=%d fail=%d / wV5066ポーリング読み %d回\n", ok, fail, n5066);
}

// SCDT(信号ロック)候補ビット待ち: `9d`バンク reg 0x12 の bit0x80 が
// 0(=信号あり,実測ON=0x11) になるまでポーリングする。
// 2026-07-09 kusq協力の信号あり/なし差分実験で特定(FINDINGS.md参照)。
static int wait_for_lock(libusb_device_handle* h, int timeout_ms, int poll_interval_ms) {
    unsigned char setup[3] = {0x9d, 0x01, 0x12};
    unsigned char resp[4];
    int waited = 0;
    while (waited < timeout_ms) {
        int r1 = libusb_control_transfer(h, 0x40, 192, 0x5066, 0, setup, 3, 500);
        int r2 = libusb_control_transfer(h, 0xc0, 192, 0x5066, 0, resp, 1, 500);
        if (r1 >= 0 && r2 >= 1) {
            fprintf(stderr, "[lock] 9d:0x12 = 0x%02x (%s)\n", resp[0],
                    (resp[0] & 0x80) ? "信号なし" : "ロック済み!");
            if (!(resp[0] & 0x80)) return 1;   // bit7クリア = ロック
        } else {
            fprintf(stderr, "[lock] ポーリング失敗 r1=%d r2=%d\n", r1, r2);
        }
        struct timeval tv = {0, poll_interval_ms * 1000};
        libusb_handle_events_timeout(NULL, &tv);
        waited += poll_interval_ms;
    }
    return 0; // タイムアウト
}

// --- ポストストリーム点火バースト（frame8637-13573） ---
// alt2でisoを開いた"後"にWindowsが送る映像パイプ有効化コマンド群。
// これが実際にIT6802Eフォーマッタ+FX3 DMAを起動する本体（2026-07-09 workflow解析で特定）。
typedef struct {
    double t;                 // frame.time_relative
    unsigned char brt, br;    // bmRequestType, bRequest
    unsigned short wv, wi, wl;
    unsigned char data[80];
    int dlen, is_out;
} BurstCmd;
static BurstCmd g_burst[2048];
static int g_nburst = 0;

static void load_burst(const char* path) {
    FILE* f = fopen(path, "r");
    if (!f) { fprintf(stderr, "[burst] tsv開けない: %s\n", path); return; }
    char line[8192]; int first = 1;
    while (fgets(line, sizeof(line), f) && g_nburst < 2048) {
        if (first) { first = 0; continue; }
        char c_frame[32], c_time[32], c_brt[16], c_br[16], c_wv[16], c_wi[16], c_wl[16], c_data[8000];
        c_data[0] = 0;
        int nf = sscanf(line, "%31[^\t]\t%31[^\t]\t%15[^\t]\t%15[^\t]\t%15[^\t]\t%15[^\t]\t%15[^\t]\t%7999[^\t\n]",
                        c_frame, c_time, c_brt, c_br, c_wv, c_wi, c_wl, c_data);
        if (nf < 7) continue;
        BurstCmd* b = &g_burst[g_nburst];
        b->t = atof(c_time);
        b->brt = (unsigned char)strtol(c_brt, NULL, 16);
        b->br  = (unsigned char)strtol(c_br, NULL, 10);
        b->wv  = (unsigned short)strtol(c_wv, NULL, 0);
        b->wi  = (unsigned short)strtol(c_wi, NULL, 0);
        b->wl  = (unsigned short)strtol(c_wl, NULL, 10);
        b->is_out = (b->brt & 0x80) == 0;
        b->dlen = (nf >= 8 && c_data[0]) ? hex2bin(c_data, b->data, sizeof(b->data)) : 0;
        g_nburst++;
    }
    fclose(f);
    fprintf(stderr, "[burst] %d コマンド ロード\n", g_nburst);
}

// ======================================================================
// SECTION 4: main (デバイス open → 呪文再生 → iso キャプチャ → 統計出力)
// ======================================================================
// TODO: パススルー (HDMI ループスルー) の恒久有効化と、それを維持したまま
// キャプチャする経路の両立が未解決。現状は pt-only モードで一時的に維持のみ。
int main(int argc, char** argv) {
    // stderr を無バッファに (SCHED_FIFO でリアルタイム優先度にすると
    // ブロックバッファになってログが即表示されない問題の対策)
    setvbuf(stderr, NULL, _IONBF, 0);
    setvbuf(stdout, NULL, _IONBF, 0);
    int read_sec = argc > 1 ? atoi(argv[1]) : 6;
    // 秒数 0 or 負数を「実用上無限 (~68 年)」に扱う
    // 注: 100 年 = 3,153,600,000 は int overflow なので INT_MAX-3600 で安全
    if (read_sec <= 0) read_sec = 2147480047;  // INT_MAX - 3600, ~68 years
    int alt = argc > 2 ? atoi(argv[2]) : 2;
    // 5番目の引数が "pt" ならパススルー専用モード（iso張らず、9a burstだけ撃って維持）
    int passthrough_only = (argc > 5 && strcmp(argv[5], "pt") == 0);
    // ラベルは 5番目 (pt モードでは 6番目)
    if (passthrough_only && argc > 6) argv[4] = argv[6];

    // 2026-07-10 SCHED_FIFO を撤去: カクツキ改善効果が実測ゼロだった一方、
    // このプロセスがCPUを独占しlibusbの内部処理(別スレッド/カーネルワーカー)が
    // スケジューリングされず replay_spell が "open/claim OK" 以降ハングする
    // 重大な副作用が判明(再現確認済み, CPU 0%で停止)。mlockallのみ残す(害なし)。
    if (mlockall(MCL_CURRENT | MCL_FUTURE) < 0) fprintf(stderr, "[main] mlockall 失敗: %s(続行)\n", strerror(errno));

    if (libusb_init(NULL) < 0) { fprintf(stderr, "libusb_init失敗\n"); return 1; }
    libusb_device_handle* h = libusb_open_device_with_vid_pid(NULL, VID, PID);
    if (!h) { fprintf(stderr, "デバイスopen失敗 (0fd9:0074)\n"); return 1; }
    libusb_set_auto_detach_kernel_driver(h, 1);
    // 2026-07-10 デバイス強制リセット: 前回異常終了後や物理抜き差し後にI2Cバスがロックしたり
    // 内部状態が乱れる問題への対策 (実測: reset無しだと 9d:0x12 が0x9d返しで100%空パケット)
    // HD60S_NO_RESET=1 で skip (passthrough 状態を壊したくない時用)
    const char* env_no_reset = getenv("HD60S_NO_RESET");
    int no_reset = (env_no_reset && env_no_reset[0] && env_no_reset[0] != '0' && env_no_reset[0] != 'n' && env_no_reset[0] != 'N');
    int rst = no_reset ? 0 : libusb_reset_device(h);
    if (no_reset) fprintf(stderr, "[main] reset 省略 (HD60S_NO_RESET)\n");
    if (rst == LIBUSB_ERROR_NOT_FOUND) {
        // reset後にデバイスIDが変わることがある→再オープン
        fprintf(stderr, "[main] reset後デバイス再列挙、再オープン中...\n");
        libusb_close(h);
        int retry = 0;
        while (retry++ < 20) {
            usleep(200000);
            h = libusb_open_device_with_vid_pid(NULL, VID, PID);
            if (h) break;
        }
        if (!h) { fprintf(stderr, "reset後デバイス再open失敗\n"); return 1; }
        libusb_set_auto_detach_kernel_driver(h, 1);
    } else if (rst < 0) {
        fprintf(stderr, "[main] reset警告 (%d) - 続行\n", rst);
    } else {
        fprintf(stderr, "[main] リセット成功\n");
    }
    if (libusb_set_configuration(h, 1) < 0) fprintf(stderr, "set_config警告\n");
    if (libusb_claim_interface(h, 0) < 0) { fprintf(stderr, "claim失敗\n"); return 1; }
    fprintf(stderr, "[main] open/claim OK (リセット後)\n");

    // 差分観測ログ: 5番目の引数をラベルに captures/reads-<label>.tsv へ全IN読みを記録
    const char* label = argc > 4 ? argv[4] : NULL;
    if (label) {
        char lp[256]; snprintf(lp, sizeof(lp), "captures/reads-%s.tsv", label);
        g_rdlog = fopen(lp, "w");
        if (g_rdlog) { fprintf(g_rdlog, "idx\twValue\tsetupOUT\tresponse\n"); fprintf(stderr, "[main] 読みログ: %s\n", lp); }
    }

    // HD60S_SKIP_INIT=1 で init 呪文 replay + burst を丸ごとスキップ
    // (HD60S が既に別セッションで init 済みなら、我々が触ると passthrough が壊れる仮説)
    const char* env_skip_init = getenv("HD60S_SKIP_INIT");
    int skip_init = (env_skip_init && env_skip_init[0] && env_skip_init[0] != '0' && env_skip_init[0] != 'n' && env_skip_init[0] != 'N');
    if (!skip_init) {
        // 環境変数 HD60S_INIT_TSV で init TSV path を切り替え可能
        const char* init_tsv = getenv("HD60S_INIT_TSV");
        if (!init_tsv || !*init_tsv) init_tsv = "analysis/init-timed.tsv";
        fprintf(stderr, "[main] init: %s\n", init_tsv);
        replay_spell(h, init_tsv);
        if (g_rdlog) { fclose(g_rdlog); g_rdlog = NULL; }
    } else {
        fprintf(stderr, "[main] HD60S_SKIP_INIT=1: init replay 省略\n");
    }

    // HDMI受信の信号ロックを"実際に見て"待つ(2026-07-09特定: 9d:0x12 bit0x80)。
    // 引数3が負数なら旧来の固定秒数待ちにフォールバック(比較用)。
    int lock_wait = argc > 3 ? atoi(argv[3]) : 4;
    if (lock_wait < 0) {
        int fixed = -lock_wait;
        fprintf(stderr, "[main] (固定)HDMIロック待ち %d秒...\n", fixed);
        for (int s = 0; s < fixed; s++) { struct timeval t={1,0}; libusb_handle_events_timeout(NULL,&t); }
    } else {
        fprintf(stderr, "[main] HDMIロック待ち(実検知, 最大%ds)...\n", lock_wait);
        int locked = wait_for_lock(h, lock_wait * 1000, 200);
        fprintf(stderr, locked ? "[main] ロック検出！\n" : "[main] タイムアウト(未ロックのまま続行)\n");
    }

    // 診断(rank2): 入力タイミングレジスタを読み解像度確認(9d bank)。期待=1920x1080@60。
    {
        struct { const char* name; unsigned char hi, lo; } regs[] = {
            {"HActive", 0x29, 0x28}, {"HTotal", 0x6b, 0x6a}, {"VTotal", 0x5c, 0x5b},
        };
        for (int k = 0; k < 3; k++) {
            unsigned char o[3], in[1]; int v[2];
            for (int p = 0; p < 2; p++) {
                o[0]=0x9d; o[1]=0x01; o[2]= p ? regs[k].lo : regs[k].hi;
                libusb_control_transfer(h, 0x40, 192, 0x5066, 0, o, 3, 500);
                in[0]=0; libusb_control_transfer(h, 0xc0, 192, 0x5066, 0, in, 1, 500);
                v[p]=in[0];
            }
            fprintf(stderr, "[res] %s = %d (0x%02x%02x)\n", regs[k].name, (v[0]<<8)|v[1], v[0], v[1]);
        }
    }

    // パススルー専用モード: iso は張らず、poststream-full の全書込を pcap タイミング通りに撃つ。
    // 6番目引数が "release" なら poststream-full 撃った後に「IT66121解放シーケンス」10コマンド追加。
    // 解放シーケンス = pcap末尾に無い「AV_MUTE解除+SW_RST全解除+AFE fire+HDCP無効」の必須尾追い。
    // (2026-07-09 深夜 IT66121公開ドライバ(mainline it66121.c / HDZero / fl2000_drm)調査で判明)
    int do_release = (argc > 6 && strcmp(argv[6], "release") == 0);
    if (passthrough_only) {
        const char* pt_tsv = "analysis/poststream-full.tsv";
        if (argc > 6 && strcmp(argv[6], "nohdcp") == 0) {
            pt_tsv = "analysis/poststream-nohdcp.tsv";
            fprintf(stderr, "[pt-only] HDCP無効化モード\n");
        }
        fprintf(stderr, "[pt-only] %s を発火（iso 張らない）\n", pt_tsv);
        load_burst(pt_tsv);
        // 相対タイミングで発火
        double t0 = g_nburst ? g_burst[0].t : 0;
        double start = now_s();
        int ok = 0, fail = 0;
        for (int i = 0; i < g_nburst; i++) {
            BurstCmd* b = &g_burst[i];
            double target = b->t - t0;
            double now = now_s() - start;
            if (target > now) usleep((useconds_t)((target - now) * 1e6));
            unsigned char inbuf[80]; int r;
            if (b->is_out) r = libusb_control_transfer(h, b->brt, b->br, b->wv, b->wi, b->data, b->dlen, 1000);
            else r = libusb_control_transfer(h, b->brt, b->br, b->wv, b->wi, inbuf, b->wl, 1000);
            if (r < 0) fail++; else ok++;
        }
        fprintf(stderr, "[pt-only] 発火完了 ok=%d fail=%d elapsed=%.2fs\n", ok, fail, now_s() - start);

        // 診断: IT66121 の 0x0E SYS_STATUS を読む (HPD/VID_STABLE 確認)
        {
            unsigned char setup[3] = {0x9b, 0x01, 0x0e};
            unsigned char val = 0;
            libusb_control_transfer(h, 0x40, 192, 0x5066, 0, setup, 3, 500);
            libusb_control_transfer(h, 0xc0, 192, 0x5066, 0, &val, 1, 500);
            fprintf(stderr, "[diag] IT66121 SYS_STATUS(0x0E)=0x%02x HPD=%d VID_STABLE=%d\n",
                    val, !!(val & 0x40), !!(val & 0x10));
        }

        // IT66121 解放シーケンス (2026-07-09 深夜, ITE公開ドライバ調査で判明)
        // pcap末尾は AV_MUTE ON+SW_RST 残置で終わってた=Windowsは追加コマンドで解放してた
        if (do_release) {
            fprintf(stderr, "[pt-only] IT66121 解放シーケンス発火 (AV_MUTE OFF, SW_RST 全解除, AFE fire, HDCP無効)\n");
            static const unsigned char release_seq[][3] = {
                {0x9a, 0x0f, 0x00},  // bank 0 選択(保険)
                {0x9a, 0x04, 0x00},  // SW_RST 全部解除 (VID/REF/AUD/AREF/HDCP)
                {0x9a, 0x62, 0x18},  // AFE_XP: RESETB=1, ENO=1
                {0x9a, 0x64, 0x94},  // AFE_IP: RESETB=1, GAINBIT=1, CKSEL_1 (>80MHz)
                {0x9a, 0x68, 0x00},  // LOWCLK clear
                {0x9a, 0x61, 0x00},  // AFE FIRE: RST=0 PWD=0 = TMDS 出力開始
                {0x9a, 0x20, 0x00},  // HDCP CPDESIRED=0 = 認証不要
                {0x9a, 0xc0, 0x01},  // HDMI モード (bit0=1)
                {0x9a, 0xc1, 0x00},  // AV MUTE OFF
                {0x9a, 0xc6, 0x03},  // Packet Gen ON + RPT
            };
            int nr = sizeof(release_seq) / 3;
            for (int i = 0; i < nr; i++) {
                int r = libusb_control_transfer(h, 0x40, 192, 0x5066, 0,
                                                (unsigned char*)release_seq[i], 3, 500);
                fprintf(stderr, "[release] %d/%d 9a %02x %02x r=%d\n", i+1, nr,
                        release_seq[i][1], release_seq[i][2], r);
                usleep(2000);
            }
            // 再診断
            unsigned char setup2[3] = {0x9b, 0x01, 0x0e};
            unsigned char val2 = 0;
            libusb_control_transfer(h, 0x40, 192, 0x5066, 0, setup2, 3, 500);
            libusb_control_transfer(h, 0xc0, 192, 0x5066, 0, &val2, 1, 500);
            fprintf(stderr, "[diag] SYS_STATUS after release = 0x%02x HPD=%d VID_STABLE=%d\n",
                    val2, !!(val2 & 0x40), !!(val2 & 0x10));
        }

        fprintf(stderr, "[pt-only] %d秒維持中...モニタ確認して\n", read_sec);
        for (int s = 0; s < read_sec; s++) sleep(1);
        libusb_release_interface(h, 0);
        libusb_close(h);
        libusb_exit(NULL);
        return 0;
    }

    // 点火バーストをロード(iso開始後に相対タイミングで発行)
    // HD60S_SKIP_BURST=1 で poststream burst 丸ごとスキップ (passthrough 保護)
    const char* env_skip_burst = getenv("HD60S_SKIP_BURST");
    int skip_burst = skip_init || (env_skip_burst && env_skip_burst[0] && env_skip_burst[0] != '0' && env_skip_burst[0] != 'n' && env_skip_burst[0] != 'N');
    if (!skip_burst) {
        const char* burst_tsv = getenv("HD60S_BURST_TSV");
        if (!burst_tsv || !*burst_tsv) burst_tsv = "analysis/poststream-full.tsv";
        load_burst(burst_tsv);
    } else {
        fprintf(stderr, "[main] burst load 省略\n");
    }

    // 🔥 PASSTHROUGH PRE-ISO ENABLE (2026-07-11 Fable + kusq webcam 実験)
    // Windows pcap の enable trio 1 回目は **iso 開始 384ms 前** に発射される。
    // つまり init TSV 完了後、alt=2 の前 = ここで発射する必要ある。常に有効。
    {
        fprintf(stderr, "[pt-pre-iso] enable trio 1st fire (pre-alt=2)...\n");
        // (0) aa 12 34 90 05 00
        unsigned char t0[6] = {0xaa, 0x12, 0x34, 0x90, 0x05, 0x00};
        libusb_control_transfer(h, 0x40, 0xC0, 0x5066, 0, t0, 6, 1000);
        // (1) aa 12 34 90 03 00
        unsigned char t1[6] = {0xaa, 0x12, 0x34, 0x90, 0x03, 0x00};
        libusb_control_transfer(h, 0x40, 0xC0, 0x5066, 0, t1, 6, 1000);
        // (2) d4 00 04 03 (CPLD routing enable)
        unsigned char t2[4] = {0xd4, 0x00, 0x04, 0x03};
        libusb_control_transfer(h, 0x40, 0xC0, 0x5066, 0, t2, 4, 1000);
        // (3) 5098 data=[0x20, 0x05] LE (Windows でも観測)
        unsigned char t3[2] = {0x20, 0x05};
        libusb_control_transfer(h, 0x40, 0xC0, 0x5098, 0, t3, 2, 1000);
        usleep(50000);  // 50ms sleep before alt=2 (Windows-observed)
    }

    if (libusb_set_interface_alt_setting(h, 0, alt) < 0) fprintf(stderr, "set_alt(%d)失敗\n", alt);
    else fprintf(stderr, "[main] alt=%d 選択OK\n", alt);
    // パススルー対策(H1): SET_INTERFACE → 初burstまで pcap実測 103ms 空ける (FX3 GPIF 再初期化とI2C競合回避)
    { struct timeval td={0, 120000}; libusb_handle_events_timeout(NULL, &td); }
    fprintf(stderr, "[main] iso開始 %d秒\n", read_sec);

    // v4l2loopback (/dev/video42) をオープン。失敗しても続行(生ストリームだけ保存)。
    g_v4l_fd = v4l2_open("/dev/video42");
    // ALSA snd-aloop hw:10,0 をオープン。環境変数 HD60S_ALSA_DEV でデバイス指定可能。
    const char* alsa_dev = getenv("HD60S_ALSA_DEV");
    if (!alsa_dev) alsa_dev = "hw:10,0";
    audio_open(alsa_dev);

    outf = fopen("captures/stream-iso.bin", "wb");
    if (!outf) fprintf(stderr, "生ストリーム出力ファイル開けず(続行)\n");

    // firmware解析結果に基づき 0x509c (MCU bridge) audio init sequence を replay
    // pcap init-timed t=6.5-7.2s から抽出した 236個の 2byte writes
    // 環境変数 HD60S_509C=1 で有効化
    // 環境変数は "1"/"yes"/"on" で有効化 (getenv() != NULL だと "0" でも ON になり直感反するため)
    const char* env_509c = getenv("HD60S_509C");
    if (env_509c && env_509c[0] && env_509c[0] != '0' && env_509c[0] != 'n' && env_509c[0] != 'N') {
        fprintf(stderr, "[509c-init] MCU audio init sequence (236 writes)...\n");
    /* total 236 2-byte writes */
    static const unsigned short audio_init_seq[] = {
        0x0000, 0x1308, 0x0000, 0x0000, 0xb702, 0x0000, 0x416f, 0x0000, 0xb800, 0x0001, 0x0f02, 0x0001,
        0x1630, 0x0001, 0x1700, 0x0001, 0x1800, 0x0001, 0x1900, 0x0001, 0x1a50, 0x0001, 0x0001, 0x2a07,
        0x0002, 0x0803, 0x0001, 0x2500, 0x0001, 0x2600, 0x0001, 0x0001, 0x0001, 0x0001, 0x0001, 0x0001,
        0x2440, 0x0001, 0x0001, 0x3080, 0x0001, 0x3100, 0x0001, 0x3200, 0x0000, 0xb014, 0x0000, 0x0000,
        0xae04, 0x0000, 0xad05, 0x0000, 0xb1c0, 0x0000, 0xb200, 0x0000, 0xb300, 0x0000, 0xb455, 0x0000,
        0x0000, 0xb454, 0x0002, 0x0161, 0x0002, 0x02f5, 0x0002, 0x0002, 0x0302, 0x0002, 0x0401, 0x0002,
        0x0500, 0x0002, 0x0608, 0x0002, 0x1c1a, 0x0002, 0x1d00, 0x0002, 0x1e00, 0x0002, 0x1f00, 0x0002,
        0x0002, 0x25a2, 0x0002, 0x0002, 0x02f5, 0x0002, 0x0002, 0x0704, 0x0002, 0x17c0, 0x0002, 0x19ff,
        0x0002, 0x1aff, 0x0002, 0x1bfc, 0x0002, 0x2000, 0x0002, 0x0002, 0x2100, 0x0002, 0x2226, 0x0002,
        0x2700, 0x0002, 0x0002, 0x2ea1, 0x0000, 0x0000, 0xab15, 0x0000, 0x0000, 0xac95, 0x0000, 0x0000,
        0xb702, 0x0000, 0xb810, 0x0000, 0xb800, 0x0002, 0x07f4, 0x0002, 0x0704, 0x0000, 0x5189, 0x0000,
        0x0000, 0xb700, 0x0000, 0xb700, 0x0002, 0x0002, 0x0161, 0x0002, 0x0002, 0x0401, 0x0002, 0x0608,
        0x0002, 0x0002, 0x0928, 0x0000, 0x0000, 0x5420, 0x0000, 0x0000, 0xac95, 0x0000, 0x0000, 0x0080,
        0x0000, 0x0000, 0xce80, 0x0000, 0x0000, 0xcf02, 0x0000, 0x0000, 0x0000, 0x0080, 0x0080, 0x0080,
        0xd000, 0x0080, 0xcf00, 0x0002, 0x0000, 0x0000, 0xab15, 0x0000, 0x0000, 0xac95, 0x0000, 0xad05,
        0x0000, 0x1e11, 0x0000, 0x1f01, 0x0000, 0x9c9f, 0x0000, 0x9b0b, 0x0000, 0x9674, 0x0000, 0x9503,
        0x0000, 0xa22c, 0x0000, 0xa101, 0x0000, 0x9a94, 0x0000, 0x9978, 0x0000, 0x942d, 0x0000, 0x9308,
        0x0000, 0xa040, 0x0000, 0x9f7f, 0x0000, 0x9eb3, 0x0000, 0x9d79, 0x0000, 0x9822, 0x0000, 0x977e,
        0x0000, 0xa42d, 0x0000, 0xa308, 0x0000, 0xa600, 0x0000, 0xa520, 0x0000, 0xa800, 0x0000, 0xa700,
        0x0000, 0xaa00, 0x0000, 0xa920, 0x0000, 0x0001, 0x0002, 0x27ff,
    };
        int seq_len = sizeof(audio_init_seq)/sizeof(audio_init_seq[0]);
        int ok = 0;
        for (int u = 0; u < seq_len; u++) {
            unsigned char d[2] = { audio_init_seq[u] & 0xff, (audio_init_seq[u] >> 8) & 0xff };
            int r = libusb_control_transfer(h, 0x40, 0xC0, 0x509c, 0, d, 2, 100);
            if (r == 2) ok++;
            usleep(500);  // pcap 実測 500us 間隔
        }
        fprintf(stderr, "[509c-init] %d/%d ok\n", ok, seq_len);
    }

    // IT6802E 0x94 bank audio unmute (Fable 発見 + IT6604 register spec 参考)
    // reg 0x87 (HWMUTE_CTRL) と reg 0x89 (TRISTATE_CTRL) を叩いて I2S 出力 untri-state
    // 環境変数 HD60S_AUDIO=1 で有効化
    const char* env_audio = getenv("HD60S_AUDIO");
    if (env_audio && env_audio[0] && env_audio[0] != '0' && env_audio[0] != 'n' && env_audio[0] != 'N') {
        fprintf(stderr, "[audio-unmute] IT6802E (0x94 bank) audio path unmute...\n");
        // IT6802E に一連のコマンドを送る (bank select → HWMUTE_CTRL clear → TRISTATE_CTRL clear)
        struct { unsigned char slave, reg, val; const char* name; } audio_unmute[] = {
            // reg 0x0f はページ選択 register (Fable 発見)
            {0x94, 0x0f, 0x02, "bank2 (audio) select"},
            // reg 0x87 (REG_RX_HWMUTE_CTRL): bit3=HW_MUTE_EN, bit4=MUTE_CLR
            //   = 0x10 → クリア (bit4 立てて bit3 クリア)
            {0x94, 0x87, 0x10, "reg 0x87 HWMUTE clear + disable"},
            // reg 0x89 (REG_RX_TRISTATE_CTRL): 全 I2S/SPDIF を untri-state
            //   = 0x00 → 全 clear
            {0x94, 0x89, 0x00, "reg 0x89 TRISTATE untri all"},
        };
        for (int u = 0; u < (int)(sizeof(audio_unmute)/sizeof(audio_unmute[0])); u++) {
            unsigned char w[3] = {audio_unmute[u].slave, audio_unmute[u].reg, audio_unmute[u].val};
            int r = libusb_control_transfer(h, 0x40, 0xC0, 0x5066, 0, w, 3, 200);
            fprintf(stderr, "  %s = %d\n", audio_unmute[u].name, r);
            usleep(2000);
        }
    }

    // IT6802E audio unmute V2 — 正しい register (IT6802 register spec より)
    // 環境変数 HD60S_AUDIO_V2=1 で有効化
    // Fable が参考にした IT6604 spec は reg 0x87 だが、IT6802 (実際の HD60S 用チップ) の
    // REG_RX_HWMuteCtrl は 0x7D。それ以外の関連 reg も一緒に叩く。
    const char* env_audio_v2 = getenv("HD60S_AUDIO_V2");
    if (env_audio_v2 && env_audio_v2[0] && env_audio_v2[0] != '0' && env_audio_v2[0] != 'n' && env_audio_v2[0] != 'N') {
        fprintf(stderr, "[audio-unmute-v2] IT6802E (正しい reg) audio path unmute...\n");
        struct { unsigned char slave, reg, val; const char* name; } audio_unmute_v2[] = {
            // page select — ページ2/audio bank へ
            {0x94, 0x0f, 0x02, "bank2 (audio) select"},
            // REG_RX_HWMuteCtrl = 0x7D
            //   bit3=HWMuteEn (0=disable HW mute), bit4=HWMuteClr (1=clear mute)
            //   0x10 = bit4 set, bit3 clear
            {0x94, 0x7d, 0x10, "reg 0x7D REG_RX_HWMuteCtrl clear"},
            // REG_RX_074 (reg 0x74): bit2=Force_AVMute (0=clear), bit3=AVMute_Value (0)
            //   全部 0 でクリア
            {0x94, 0x74, 0x00, "reg 0x74 Force_AVMute clear"},
            // REG_RX_0A8 (reg 0xA8): bit0=P0_AVMUTE, bit4=P1_AVMUTE → 全部 clear
            {0x94, 0xa8, 0x00, "reg 0xA8 AVMute (P0/P1) clear"},
            // REG_RX_07E (reg 0x7E): bit4=Force_I2SOut (=1 で強制 I2S 出力 ON)
            {0x94, 0x7e, 0x10, "reg 0x7E Force I2SOut ON"},
        };
        for (int u = 0; u < (int)(sizeof(audio_unmute_v2)/sizeof(audio_unmute_v2[0])); u++) {
            unsigned char w[3] = {audio_unmute_v2[u].slave, audio_unmute_v2[u].reg, audio_unmute_v2[u].val};
            int r = libusb_control_transfer(h, 0x40, 0xC0, 0x5066, 0, w, 3, 200);
            fprintf(stderr, "  %s = %d\n", audio_unmute_v2[u].name, r);
            usleep(2000);
        }
    }

    // 最小 unmute: 0x509c で bank2 select → reg 0x20 = 0x00
    // 環境変数 HD60S_MIN=1 で有効化
    const char* env_min = getenv("HD60S_MIN");
    if (env_min && env_min[0] && env_min[0] != '0' && env_min[0] != 'n' && env_min[0] != 'N') {
        fprintf(stderr, "[min-unmute] 0x509c 最小シーケンス (bank2 select + reg 0x20=0x00)...\n");
        // firmware解析より: dev=2 page reg 0x20 = 0x00 が unmute
        // protocol: `00 BB` = bank切替 (BB=bank), `RR VV` = reg RR = VV
        // 発行 6回 (状態安定のため)
        for (int cycle = 0; cycle < 6; cycle++) {
            unsigned char sel[2] = {0x00, 0x02};
            libusb_control_transfer(h, 0x40, 0xC0, 0x509c, 0, sel, 2, 100);
            usleep(500);
            unsigned char wr[2] = {0x20, 0x00};
            libusb_control_transfer(h, 0x40, 0xC0, 0x509c, 0, wr, 2, 100);
            usleep(500);
        }
        fprintf(stderr, "  min-unmute done\n");
    }

    // firmware解析結果に基づき IT6802E (0x9c bank) audio unmute 直接I2C書き込み
    // 環境変数 HD60S_UNMUTE=1 で有効化 (legacy)
    const char* env_unmute = getenv("HD60S_UNMUTE");
    if (env_unmute && env_unmute[0] && env_unmute[0] != '0' && env_unmute[0] != 'n' && env_unmute[0] != 'N') {
        fprintf(stderr, "[unmute] IT6802E audio unmute シーケンス投入...\n");
        struct { unsigned char reg, val; const char* name; } audio_unmute[] = {
            {0x1B, 0xFF, "reg 0x1B ch enable all"},
            {0x02, 0xF5, "reg 0x02 AUD_EN (bit7 set)"},
            {0x27, 0x00, "reg 0x27 output mute clear"},
            {0x07, 0x04, "reg 0x07 audio path enable"},
            {0x25, 0xA2, "reg 0x25 audio ctrl"},
        };
        for (int u = 0; u < (int)(sizeof(audio_unmute)/sizeof(audio_unmute[0])); u++) {
            unsigned char w[3] = {0x9c, audio_unmute[u].reg, audio_unmute[u].val};
            int r = libusb_control_transfer(h, 0x40, 0xC0, 0x5066, 0, w, 3, 1000);
            fprintf(stderr, "  %s = %d\n", audio_unmute[u].name, r);
            usleep(2000);
        }
        // IT66121 (0x9a) 側 audio ctrl も unmute
        struct { unsigned char reg, val; const char* name; } tx_audio[] = {
            {0xC1, 0x00, "reg 0xC1 AVMUTE clear"},
            {0xB9, 0x03, "reg 0xB9 audio InfoFrame enable"},
            {0xE0, 0x01, "reg 0xE0 audio ctrl"},
            {0xE4, 0x10, "reg 0xE4 audio config"},
        };
        for (int u = 0; u < (int)(sizeof(tx_audio)/sizeof(tx_audio[0])); u++) {
            unsigned char w[3] = {0x9a, tx_audio[u].reg, tx_audio[u].val};
            int r = libusb_control_transfer(h, 0x40, 0xC0, 0x5066, 0, w, 3, 1000);
            fprintf(stderr, "  IT66121 %s = %d\n", tx_audio[u].name, r);
            usleep(2000);
        }
    }

    struct libusb_transfer* xfrs[NUM_TRANSFERS];
    unsigned char* bufs[NUM_TRANSFERS];
    for (int i = 0; i < NUM_TRANSFERS; i++) {
        // Zerocopy DMA バッファ (usbfs mmap経由) → CPU 使用率↓、tail latency↓。
        // 失敗時は malloc にフォールバック (小型ホストで KMS が確保できない場合)。
        bufs[i] = libusb_dev_mem_alloc(h, ISO_PACKETS * ISO_PKTSIZE);
        if (!bufs[i]) bufs[i] = malloc(ISO_PACKETS * ISO_PKTSIZE);
        xfrs[i] = libusb_alloc_transfer(ISO_PACKETS);
        // timeout=0 = 無限。連続isoで有限timeoutはURBキャンセルで in-flight packet 全て empty化する罠
        libusb_fill_iso_transfer(xfrs[i], h, EP_STREAM, bufs[i],
            ISO_PACKETS * ISO_PKTSIZE, ISO_PACKETS, iso_cb, NULL, 0);
        libusb_set_iso_packet_lengths(xfrs[i], ISO_PKTSIZE);
        if (libusb_submit_transfer(xfrs[i]) == 0) inflight++;
        else fprintf(stderr, "submit%d失敗\n", i);
    }
    fprintf(stderr, "[main] iso転送 %d 本投入\n", inflight);

    // 🔍 IT66121 状態 dump (パススルー sequence 前後の切り分け用)
    // マクロは file 後半で定義されてるので inline で書く
    {
        unsigned char regs[16] = {0};
        for (int r = 0; r < 16; r++) {
            unsigned char setup[3] = {0x9b, 0x01, (unsigned char)r};
            unsigned char resp[1] = {0};
            libusb_control_transfer(h, 0x40, 0xC0, 0x5066, 0, setup, 3, 500);
            int rr = libusb_control_transfer(h, 0xC0, 0xC0, 0x5066, 0, resp, 1, 500);
            regs[r] = (rr > 0) ? resp[0] : 0xff;
            usleep(1000);
        }
        fprintf(stderr, "[TX before-pt] 0x00-0x0f:");
        for (int r = 0; r < 16; r++) fprintf(stderr, " %02x", regs[r]);
        fprintf(stderr, "  (0x04=%02x → SW_RST)\n", regs[4]);
    }

    // 🔥 HDMI PASSTHROUGH ENABLE (2026-07-11 Fable 3rd 分析)
    // 真の犯人発見: IT66121 TX (slave 0x9a) は最初から完璧。真のパススルー enable は
    // MCU (slave 0xaa magic 12 34) と CPLD (slave 0xd4) 経由の "秘密コマンド 6 発"。
    // Windows はこれを 3 回発射する。「reg 0x27 = video gate」を開いて RX→TX ルート
    // を CPLD で有効化する。
    // 前バージョン (15 writes to 0x9a) は TX しか触ってなかったので RX→TX の物理路
    // が CPLD で切れたまま = TV 無信号だった。
    {
        int pt_ok = 0, pt_fail = 0;
        #define TX_WRITE(reg, val) do { \
            unsigned char _w[3] = {0x9a, (reg), (val)}; \
            int _r = libusb_control_transfer(h, 0x40, 0xC0, 0x5066, 0, _w, 3, 500); \
            if (_r == 3) pt_ok++; else pt_fail++; \
        } while (0)

        // 🎯 真のパススルー enable シーケンス (Fable 3rd 分析)
        // Windows は 3 回発射するのでここでも 3 回発射
        fprintf(stderr, "[passthrough] TRUE enable sequence (MCU+CPLD, 6 cmds × 3 rounds)...\n");
        for (int round = 0; round < 3; round++) {
            // 1. MCU reg 0x27 "video gate" open
            {
                unsigned char w[2] = {0x27, 0x00};
                int r = libusb_control_transfer(h, 0x40, 0xC0, 0x509c, 0, w, 2, 500);
                if (r == 2) pt_ok++; else pt_fail++;
            }
            // 2. MCU RPC set-mode param 5
            {
                unsigned char w[6] = {0xaa, 0x12, 0x34, 0x90, 0x05, 0x00};
                int r = libusb_control_transfer(h, 0x40, 0xC0, 0x5066, 0, w, 6, 500);
                if (r == 6) pt_ok++; else pt_fail++;
            }
            // 3. MCU RPC set-mode param 3
            {
                unsigned char w[6] = {0xaa, 0x12, 0x34, 0x90, 0x03, 0x00};
                int r = libusb_control_transfer(h, 0x40, 0xC0, 0x5066, 0, w, 6, 500);
                if (r == 6) pt_ok++; else pt_fail++;
            }
            // 4. CPLD routing reg 0x04 = 0x03 (bit0=RX→TX, bit1=RX→FX3 両方 enable)
            {
                unsigned char w[4] = {0xd4, 0x00, 0x04, 0x03};
                int r = libusb_control_transfer(h, 0x40, 0xC0, 0x5066, 0, w, 4, 500);
                if (r == 4) pt_ok++; else pt_fail++;
            }
            // 5. CPLD keepalive
            {
                unsigned char w[4] = {0xd4, 0x00, 0x2a, 0x6e};
                int r = libusb_control_transfer(h, 0x40, 0xC0, 0x5066, 0, w, 4, 500);
                if (r == 4) pt_ok++; else pt_fail++;
            }
            // 6. CPLD keepalive
            {
                unsigned char w[4] = {0xd4, 0x00, 0x01, 0x02};
                int r = libusb_control_transfer(h, 0x40, 0xC0, 0x5066, 0, w, 4, 500);
                if (r == 4) pt_ok++; else pt_fail++;
            }
            usleep(500 * 1000);  // 500ms 間隔で 3 回
        }
        fprintf(stderr, "[passthrough] MCU/CPLD enable 統計: ok=%d fail=%d\n", pt_ok, pt_fail);
        pt_ok = 0; pt_fail = 0;

        // 🔥 0x9a TX writes 全部削除 (2026-07-11 Fable 4th 分析)
        // HD60S は既定でパススルー ON、MCU が自律で IT66121 を制御する。
        // ホストから 0x9a に書き込むと MCU 設定を壊す (friendly fire) → TMDS 不安定 →
        // TV が「省電力モーダル」に落ちる。MCU/CPLD 制御 (aa/d4) だけ残し、
        // 0x9a への書き込みは一切しない。
        (void)pt_ok; (void)pt_fail;
        #undef TX_WRITE
    }
    // 🔍 IT66121 状態 dump (パススルー sequence 直後)
    {
        unsigned char regs[16] = {0};
        for (int r = 0; r < 16; r++) {
            unsigned char setup[3] = {0x9b, 0x01, (unsigned char)r};
            unsigned char resp[1] = {0};
            libusb_control_transfer(h, 0x40, 0xC0, 0x5066, 0, setup, 3, 500);
            int rr = libusb_control_transfer(h, 0xC0, 0xC0, 0x5066, 0, resp, 1, 500);
            regs[r] = (rr > 0) ? resp[0] : 0xff;
            usleep(1000);
        }
        fprintf(stderr, "[TX after-pt] 0x00-0x0f:");
        for (int r = 0; r < 16; r++) fprintf(stderr, " %02x", regs[r]);
        fprintf(stderr, "  (0x04=%02x SW_RST, 0x0E=%02x SYS_STAT)\n", regs[4], regs[14]);
    }

    // 🔥 POST-ISO AUDIO CONFIG (2026-07-11 pcap RE breakthrough)
    // Windows sends 188 I2C writes to slave 0x9a (IT66121 TX) & 0x94 (audio bank)
    // in first 400ms after iso start. We never sent these — that's why audio dies at 100ms.
    // 環境変数 HD60S_POST_ISO_AUDIO=1 で有効化 (default off で回帰リスク低減)
    {
        const char* env_pia = getenv("HD60S_POST_ISO_AUDIO");
        int do_pia = (env_pia && env_pia[0] && env_pia[0] != '0' && env_pia[0] != 'n' && env_pia[0] != 'N');
        if (do_pia) {
            #include "post_iso_audio.inc"
            fprintf(stderr, "[post-iso-audio] firing %d I2C writes...\n", post_iso_audio_n);
            int ok_pia = 0;
            for (int u = 0; u < post_iso_audio_n; u++) {
                if (post_iso_audio[u].delay_us > 0) usleep(post_iso_audio[u].delay_us);
                unsigned char w[3] = {post_iso_audio[u].b0, post_iso_audio[u].b1, post_iso_audio[u].b2};
                int r = libusb_control_transfer(h, 0x40, 0xC0, 0x5066, 0, w, 3, 200);
                if (r == 3) ok_pia++;
            }
            fprintf(stderr, "[post-iso-audio] fired %d/%d OK\n", ok_pia, post_iso_audio_n);
        }
        // 変形: 0x94 audio bank writes だけを撃つ (IT66121 TX 系列は skip)
        // 環境変数 HD60S_POST_ISO_AUDIO94=1 で有効化
        const char* env_pia94 = getenv("HD60S_POST_ISO_AUDIO94");
        int do_pia94 = (env_pia94 && env_pia94[0] && env_pia94[0] != '0' && env_pia94[0] != 'n' && env_pia94[0] != 'N');
        if (do_pia94) {
            #include "post_iso_audio94.inc"
            fprintf(stderr, "[post-iso-audio94] firing %d writes to slave 0x94 (audio bank)...\n", post_iso_audio94_n);
            int ok94 = 0;
            for (int u = 0; u < post_iso_audio94_n; u++) {
                unsigned char w[3] = {post_iso_audio94[u].b0, post_iso_audio94[u].b1, post_iso_audio94[u].b2};
                int r = libusb_control_transfer(h, 0x40, 0xC0, 0x5066, 0, w, 3, 200);
                if (r == 3) ok94++;
                usleep(600);
            }
            fprintf(stderr, "[post-iso-audio94] %d/%d OK\n", ok94, post_iso_audio94_n);
        }
    }

    // iso をポンプしながら、点火バーストを相対タイムスタンプ準拠で発行する。
    // 単一スレッド: libusb_control_transfer は内部で iso 転送もポンプするので iso は止まらない。
    double burst_t0 = g_nburst ? g_burst[0].t : 0;
    double start = now_s();
    int bi = 0, bok = 0, bfail = 0;
    struct timeval tv = {0, 10000}; // 10ms 刻み
    const char* env_pt = getenv("HD60S_PT_LOOP");
    int pt_loop = (env_pt && env_pt[0] && env_pt[0] != '0' && env_pt[0] != 'n' && env_pt[0] != 'N');
    double last_pt_fire = 0.0;
    int pt_fires = 0;
    while (now_s() - start < read_sec && inflight > 0) {
        double el = now_s() - start;
        while (bi < g_nburst && (g_burst[bi].t - burst_t0) <= el) {
            BurstCmd* b = &g_burst[bi];
            unsigned char inbuf[80]; int r;
            if (b->is_out)
                r = libusb_control_transfer(h, b->brt, b->br, b->wv, b->wi, b->data, b->dlen, 1000);
            else
                r = libusb_control_transfer(h, b->brt, b->br, b->wv, b->wi, inbuf, b->wl, 1000);
            if (r < 0) bfail++; else bok++;
            bi++;
        }
        // AUDIO KEEPALIVE V2: 2026-07-11 Opus 4.8 サブエージェント発見。
        // Windows steady-state は 227コマンドの keepalive cycle を 163ms 周期で firing.
        // 環境変数 HD60S_AUDIO_KA=1 で有効化。keepalive TSV を別バッファに読み込んで再送。
        static double last_ka_fire = 0.0;
        static int ka_fires = 0;
        static BurstCmd g_ka[512];
        static int g_nka = -1;  // -1 = 未ロード
        const char* env_ka = getenv("HD60S_AUDIO_KA");
        int ka_loop = (env_ka && env_ka[0] && env_ka[0] != '0');
        if (ka_loop && g_nka < 0) {
            // 初回: TSV load (別関数使いたいが load_burst は g_burst を潰すので inline)
            FILE* fka = fopen("analysis/keepalive-cycle-v2.tsv", "r");
            if (!fka) { fprintf(stderr, "[ka] keepalive-cycle-v2.tsv 開けず、KA無効化\n"); g_nka = 0; ka_loop = 0; }
            else {
                char line[8192]; int first = 1; g_nka = 0;
                while (fgets(line, sizeof(line), fka) && g_nka < 512) {
                    if (first) { first = 0; continue; }
                    char cf[32], ct[32], cbrt[16], cbr[16], cwv[16], cwi[16], cwl[16], cd[8000];
                    cd[0] = 0;
                    int nf = sscanf(line, "%31[^\t]\t%31[^\t]\t%15[^\t]\t%15[^\t]\t%15[^\t]\t%15[^\t]\t%15[^\t]\t%7999[^\t\n]",
                                    cf, ct, cbrt, cbr, cwv, cwi, cwl, cd);
                    if (nf < 7) continue;
                    BurstCmd* c = &g_ka[g_nka];
                    c->t = 0; c->brt = (unsigned char)strtol(cbrt, NULL, 0);
                    c->br = (unsigned char)strtol(cbr, NULL, 10);
                    c->wv = (unsigned short)strtol(cwv, NULL, 0);
                    c->wi = (unsigned short)strtol(cwi, NULL, 0);
                    c->wl = (unsigned short)strtol(cwl, NULL, 10);
                    c->is_out = (c->brt & 0x80) == 0;
                    c->dlen = 0;
                    if (c->is_out && nf >= 8 && cd[0]) {
                        c->dlen = hex2bin(cd, c->data, sizeof(c->data));
                    }
                    g_nka++;
                }
                fclose(fka);
                fprintf(stderr, "[ka] keepalive-cycle-v2.tsv: %d commands loaded\n", g_nka);
            }
        }
        // KA fires INDEPENDENT of burst completion — audio dies at 100ms so we can't wait for burst
        if (ka_loop && g_nka > 0 && (el - last_ka_fire) >= 0.163) {
            unsigned char inbuf[80];
            int ka_ok = 0;
            for (int k = 0; k < g_nka; k++) {
                BurstCmd* c = &g_ka[k];
                int r;
                if (c->is_out) r = libusb_control_transfer(h, c->brt, c->br, c->wv, c->wi, c->data, c->dlen, 100);
                else r = libusb_control_transfer(h, c->brt, c->br, c->wv, c->wi, inbuf, c->wl, 100);
                if (r >= 0) ka_ok++;
            }
            last_ka_fire = el;
            ka_fires++;
            if (ka_fires <= 3) fprintf(stderr, "[ka] cycle #%d fired: %d/%d ok\n", ka_fires, ka_ok, g_nka);
        }

        // 🔥 IT6802 AUDIO RECOVERY LOOP (2026-07-11 Fable + FIX_ID_023 breakthrough)
        // IT6802 の AudioFsCal + aud_fiforst + Force FS 相当を再現。
        // 100ms周期で HW mute解除 + 48kHz強制 + I2S untri-state を再送。
        // HD60S_IT6802_RECOVER=1 で有効化。IT6802 access = I2C slave 0x94 (write) bank 0
        static double last_it6802_rec = 0.0;
        static int it6802_rec_fires = 0;
        const char* env_rec = getenv("HD60S_IT6802_RECOVER");
        int do_rec = (env_rec && env_rec[0] && env_rec[0] != '0' && env_rec[0] != 'n' && env_rec[0] != 'N');
        static double rec_interval = -1.0;
        if (rec_interval < 0) {
            const char* env_int = getenv("HD60S_RECOVER_MS");
            rec_interval = (env_int && atoi(env_int) > 0) ? atoi(env_int) / 1000.0 : 0.100;
        }
        // t=0 でも即発火 (last_it6802_rec == 0.0 で初回、初期無音を潰す)
        if (do_rec && (last_it6802_rec == 0.0 || (el - last_it6802_rec) >= rec_interval)) {
            #define IT6802W(reg, val) do { \
                unsigned char _w[3] = {0x94, (reg), (val)}; \
                libusb_control_transfer(h, 0x40, 0xC0, 0x5066, 0, _w, 3, 100); \
            } while (0)
            // 1) select bank 0 (audio channel status is bank 2 but recovery regs are bank 0)
            IT6802W(0x0f, 0x00);
            // 2) HW mute clear: REG_RX_HWMuteCtrl(0x7D):  bit4=HWMuteClr, bit5=HWAudMuteClrMode
            IT6802W(0x7d, 0x30);   // set both
            IT6802W(0x7d, 0x00);   // clear both
            // 3) aud_fiforst: REG_RX_074(0x74) mute I2S/WS/SPDIF (bits 0x0c) then clear
            //    base 0xA0 (i2s+SPDIF enable) + 0x0c (mute) = 0xac; then 0xA0 (clear mute)
            IT6802W(0x74, 0xac);
            usleep(2000);
            IT6802W(0x74, 0xa0);
            // 4) audio logic reset: REG_RX_010(0x10) bit1 = pulse
            //    default 0x00 → 0x02 → 0x00
            IT6802W(0x10, 0x02);
            IT6802W(0x10, 0x00);
            // 5) Force FS = 48kHz (FIX_ID_023): REG_RX_074 bit6 + REG_RX_07B x 4
            IT6802W(0x74, 0xe0);   // 0xA0 base + 0x40 (B_Force_FS)
            IT6802W(0x7b, 0x02);   // B_48K
            IT6802W(0x7b, 0x02);
            IT6802W(0x7b, 0x02);
            IT6802W(0x7b, 0x02);
            // 5b) REG_RX_075 = 0x40: Audio 24bit → 16bit conversion (ITE init default)
            IT6802W(0x75, 0x40);
            // 5c) Fable's MCLK 256fs hint: reg 0x54 bits[5:4] = 01 (HBR mode = 128fs)
            //     Trying reg 0x54 value 0x10 to set 256fs mode for correct PCM bit clock
            IT6802W(0x54, 0x10);
            // 6) REG_RX_07E: clear B_HBRSel (bit6) — this WORKS for sustained audio.
            //    (Setting bit6 breaks audio entirely — the 0x40 semantics is opposite
            //     of the ITE macro name suggests)
            IT6802W(0x7e, 0x00);
            // 7) un-tristate I2S+SPDIF: REG_RX_052 clear only B_TriI2SIO(0x0F)+B_TriSPDIF(0x10)=0x1F
            //    B_DisVAutoMute (0x20) should stay set per init. So value 0x20.
            IT6802W(0x52, 0x20);
            #undef IT6802W
            last_it6802_rec = el;
            it6802_rec_fires++;
            if (it6802_rec_fires <= 3) fprintf(stderr, "[it6802-rec] fire #%d at t=%.0fms\n", it6802_rec_fires, el*1000);
        }

        // 🔥 PASSTHROUGH KEEPALIVE (2026-07-11 Fable + kusq webcam 検証)
        // MCU/CPLD の d4 slave keepalive を 100ms 周期で発射しないと LG モニターが
        // 「省電力モーダル」に落ちる (パススルー ON→OFF の切れ目を検知される)。
        // Windows pcap では 40-300ms 周期で連続発射している。
        static double last_pt_ka = 0.0;
        static int pt_ka_fires = 0;
        if ((el - last_pt_ka) >= 0.100) {
            // enable trio + keepalive pair を毎回発射
            // aa 12 34 90 05 00
            unsigned char w0a[6] = {0xaa, 0x12, 0x34, 0x90, 0x05, 0x00};
            libusb_control_transfer(h, 0x40, 0xC0, 0x5066, 0, w0a, 6, 100);
            // aa 12 34 90 03 00
            unsigned char w0b[6] = {0xaa, 0x12, 0x34, 0x90, 0x03, 0x00};
            libusb_control_transfer(h, 0x40, 0xC0, 0x5066, 0, w0b, 6, 100);
            // d4 00 04 03 (CPLD routing)
            unsigned char w0c[4] = {0xd4, 0x00, 0x04, 0x03};
            libusb_control_transfer(h, 0x40, 0xC0, 0x5066, 0, w0c, 4, 100);
            // d4 00 2a 6e
            unsigned char w1[4] = {0xd4, 0x00, 0x2a, 0x6e};
            libusb_control_transfer(h, 0x40, 0xC0, 0x5066, 0, w1, 4, 100);
            // d4 00 01 02
            unsigned char w2[4] = {0xd4, 0x00, 0x01, 0x02};
            libusb_control_transfer(h, 0x40, 0xC0, 0x5066, 0, w2, 4, 100);
            last_pt_ka = el;
            pt_ka_fires++;
            if (pt_ka_fires <= 3) fprintf(stderr, "[pt-keepalive] fire #%d at t=%.0fms (full trio+keepalive)\n", pt_ka_fires, el*1000);
        }

        // 60B MCU BATCH RETRY: iso 中に arm batch を反復発火して audio DMA restart 試行
        // 環境変数 HD60S_BATCH_LOOP=1 で有効化、80ms 周期
        static double last_batch_fire = 0.0;
        static int batch_fires = 0;
        const char* env_batch_loop = getenv("HD60S_BATCH_LOOP");
        int batch_loop = (env_batch_loop && env_batch_loop[0] && env_batch_loop[0] != '0');
        if (batch_loop && (el - last_batch_fire) >= 0.080) {
            static const unsigned char arm_payload[60] = {
                0x55, 0x80, 0x3c, 0x00, 0x6a, 0x80, 0x0f, 0x00, 0x6b, 0x80, 0xfe, 0x00,
                0x01, 0x81, 0x07, 0x00, 0x0b, 0x82, 0xdf, 0x00, 0x0c, 0x82, 0x3f, 0x00,
                0x0e, 0x82, 0x08, 0x00, 0x48, 0x82, 0x60, 0x00, 0x9b, 0x82, 0xf0, 0x00,
                0x11, 0x82, 0xff, 0x00, 0x12, 0x82, 0xff, 0x00, 0x11, 0x82, 0xff, 0x00,
                0x12, 0x82, 0xff, 0x00, 0x0e, 0x40, 0xd0, 0x9a, 0x0e, 0x40, 0x80, 0x9a
            };
            libusb_control_transfer(h, 0x40, 0xC6, 0x0000, 0x0100, NULL, 0, 100);
            libusb_control_transfer(h, 0x40, 0xC6, 0x0032, 0x0101, (unsigned char*)arm_payload, 60, 100);
            last_batch_fire = el;
            batch_fires++;
            if (batch_fires <= 3) fprintf(stderr, "[batch] fired #%d at t=%.0fms\n", batch_fires, el*1000);
        }

        // PLL LOCK MONITOR: 2026-07-11 Fable ヒント。IT6802 の IPLL_LOCK が
        // 音声死亡時に drop してるかを 20ms 周期で monitor。
        // 環境変数 HD60S_PLL_MON=1 で有効化
        static double last_pll_mon = 0.0;
        static int pll_mon_fires = 0;
        static unsigned char pll_history[64] = {0};
        static int pll_hist_pos = 0;
        const char* env_pll_mon = getenv("HD60S_PLL_MON");
        int pll_mon = (env_pll_mon && env_pll_mon[0] && env_pll_mon[0] != '0');
        if (pll_mon && (el - last_pll_mon) >= 0.020) {
            // BREAKTHROUGH: slave 0x94 audio bank + read via 0x95 = real audio state
            // First select page 2 audio bank on slave 0x94
            static int first_time = 1;
            if (first_time) {
                unsigned char bank[3] = {0x94, 0x0f, 0x02};
                libusb_control_transfer(h, 0x40, 0xC0, 0x5066, 0, bank, 3, 30);
                first_time = 0;
            }
            unsigned char vals[6] = {0};
            const unsigned char reads[][3] = {
                {0x95, 0x01, 0xaa},  // AUDIO_CH_STAT via correct slave
                {0x95, 0x01, 0xae},  // AUD_CHSTAT3 (M_FS)
                {0x95, 0x01, 0xad},  // channel/source count
                {0x95, 0x01, 0xac},  // channel status
                {0x95, 0x01, 0xab},  // channel status 0
                {0x95, 0x01, 0xa9},  // (unknown but returned 0x11 in probe)
            };
            for (int i = 0; i < 6; i++) {
                libusb_control_transfer(h, 0x40, 0xC0, 0x5066, 0, (unsigned char*)reads[i], 3, 30);
                libusb_control_transfer(h, 0xC0, 0xC0, 0x5066, 0, &vals[i], 1, 30);
            }
            if (pll_mon_fires < 60) {
                fprintf(stderr, "[audio] t=%6.0fms AA=0x%02x(AUD_ON=%d HBR=%d LAYOUT=%d CH=%d) AE=0x%02x(Fs=%d) AD=0x%02x AC=0x%02x AB=0x%02x A9=0x%02x\n",
                        el*1000, vals[0],
                        (vals[0]>>7)&1, (vals[0]>>6)&1, (vals[0]>>4)&1, vals[0]&0x0F,
                        vals[1], vals[1]&0x0F,
                        vals[2], vals[3], vals[4], vals[5]);
            }
            last_pll_mon = el;
            pll_mon_fires++;
        }

        // AUDIO ARM RETRY: 2026-07-11 iso 中に arm sequence を反復投入。
        // 100ms で音声死ぬ→ もしかしたら arm 効果が 100ms しかもたない?
        // 環境変数 HD60S_ARM_LOOP=1 で有効化、30ms 周期
        static double last_arm_loop = 0.0;
        static int arm_loop_fires = 0;
        const char* env_arm_loop = getenv("HD60S_ARM_LOOP");
        int arm_loop = (env_arm_loop && env_arm_loop[0] && env_arm_loop[0] != '0');
        if (arm_loop && (el - last_arm_loop) >= 0.030) {
            unsigned char arm_a[4] = {0xd4, 0x00, 0x04, 0x03};
            libusb_control_transfer(h, 0x40, 0xC0, 0x5066, 0, arm_a, 4, 100);
            unsigned char arm_b[2] = {0x20, 0x05};
            libusb_control_transfer(h, 0x40, 0xC0, 0x5098, 0, arm_b, 2, 100);
            last_arm_loop = el;
            arm_loop_fires++;
        }

        // AUDIO UNMUTE-RETRY: 2026-07-11 Fable ヒント。IT6802E は ACR unlock で
        // hard-mute 発動→ read-then-clear 必要。50ms 周期で mute clear を強制書き込み。
        // 環境変数 HD60S_UNMUTE_RETRY=1 で有効化
        static double last_unmute_fire = 0.0;
        static int unmute_fires = 0;
        const char* env_unmute_retry = getenv("HD60S_UNMUTE_RETRY");
        int unmute_retry = (env_unmute_retry && env_unmute_retry[0] && env_unmute_retry[0] != '0');
        if (unmute_retry && (el - last_unmute_fire) >= 0.050) {
            // Bank2 select (audio bank)
            unsigned char sel[3] = {0x94, 0x0f, 0x02};
            libusb_control_transfer(h, 0x40, 0xC0, 0x5066, 0, sel, 3, 100);
            // Force clear HWMuteCtrl (reg 0x7D bit4 = HWMuteClr trigger)
            unsigned char w1[3] = {0x94, 0x7d, 0x10};
            libusb_control_transfer(h, 0x40, 0xC0, 0x5066, 0, w1, 3, 100);
            // Clear Force_AVMute (reg 0x74)
            unsigned char w2[3] = {0x94, 0x74, 0x00};
            libusb_control_transfer(h, 0x40, 0xC0, 0x5066, 0, w2, 3, 100);
            // Clear P0/P1_AVMUTE (reg 0xA8)
            unsigned char w3[3] = {0x94, 0xa8, 0x00};
            libusb_control_transfer(h, 0x40, 0xC0, 0x5066, 0, w3, 3, 100);
            // Force I2S output on (reg 0x7E bit4)
            unsigned char w4[3] = {0x94, 0x7e, 0x10};
            libusb_control_transfer(h, 0x40, 0xC0, 0x5066, 0, w4, 3, 100);
            last_unmute_fire = el;
            unmute_fires++;
            if (unmute_fires <= 3) fprintf(stderr, "[unmute-retry] fired #%d\n", unmute_fires);
        }

        // min-unmute-loop: burst 完了後、100ms 周期で bank2 select + reg 0x20=0x00 を再送
        // 環境変数 HD60S_MIN_LOOP=1 で有効化 (P5 で peak +71% 確認済み)
        static double last_min_fire = 0.0;
        static int min_fires = 0;
        const char* env_min_loop = getenv("HD60S_MIN_LOOP");
        int min_loop = (env_min_loop && env_min_loop[0] && env_min_loop[0] != '0');
        if (min_loop && bi >= g_nburst && (el - last_min_fire) >= 0.10) {
            unsigned char sel[2] = {0x00, 0x02};
            libusb_control_transfer(h, 0x40, 0xC0, 0x509c, 0, sel, 2, 50);
            unsigned char wr[2] = {0x20, 0x00};
            libusb_control_transfer(h, 0x40, 0xC0, 0x509c, 0, wr, 2, 50);
            last_min_fire = el;
            min_fires++;
        }

        // pt-loop: burst 完了後、120ms 周期で **完全な** passthrough keep-alive cycle を発火
        // pcap 解析: 30 commands (9a 書き, 9b/9d 読み, 0x509c MCU) を 120ms 周期で全部やる
        // 詳細は analysis/keepalive-cycle.tsv 参照
        if (pt_loop && bi >= g_nburst && (el - last_pt_fire) >= 0.12) {
            // IT66121 writes: 9a0f00, 9ac101, 9ac603 (start of cycle)
            unsigned char w1[3] = {0x9a, 0x0f, 0x00}; libusb_control_transfer(h, 0x40, 0xC0, 0x5066, 0, w1, 3, 100);
            unsigned char w2[3] = {0x9a, 0xc1, 0x01}; libusb_control_transfer(h, 0x40, 0xC0, 0x5066, 0, w2, 3, 100);
            unsigned char w3[3] = {0x9a, 0xc6, 0x03}; libusb_control_transfer(h, 0x40, 0xC0, 0x5066, 0, w3, 3, 100);
            // IT66121 read setup for 9b/reg 0x0e (SYS_STATUS)
            unsigned char rs1[3] = {0x9b, 0x01, 0x0e};
            libusb_control_transfer(h, 0x40, 0xC0, 0x5066, 0, rs1, 3, 100);
            unsigned char rb[1];
            libusb_control_transfer(h, 0xC0, 0xC0, 0x5066, 0, rb, 1, 100);
            // MCU seq (0x509c series)
            unsigned short ka_seq[] = {
                0x0000, 0x0000, 0xb3ff, 0x0002, 0x27ff, 0x0002,
                0x0002, 0x0002, 0x0002, 0x0002, 0x2000
            };
            for (int k = 0; k < (int)(sizeof(ka_seq)/sizeof(ka_seq[0])); k++) {
                unsigned char d[2] = { ka_seq[k] & 0xff, (ka_seq[k] >> 8) & 0xff };
                libusb_control_transfer(h, 0x40, 0xC0, 0x509c, 0, d, 2, 100);
                usleep(500);
            }
            // IT6802E reads: 0x11, 0x12 (interleaved with 0x509c 0x0002 - simplified)
            unsigned char rs2[3] = {0x9d, 0x01, 0x11};
            libusb_control_transfer(h, 0x40, 0xC0, 0x5066, 0, rs2, 3, 100);
            libusb_control_transfer(h, 0xC0, 0xC0, 0x5066, 0, rb, 1, 100);
            unsigned char rs3[3] = {0x9d, 0x01, 0x12};
            libusb_control_transfer(h, 0x40, 0xC0, 0x5066, 0, rs3, 3, 100);
            libusb_control_transfer(h, 0xC0, 0xC0, 0x5066, 0, rb, 1, 100);
            last_pt_fire = el;
            pt_fires++;
        }
        libusb_handle_events_timeout(NULL, &tv);
    }
    if (pt_loop) fprintf(stderr, "[pt-loop] 継続発火 %d 回\n", pt_fires);
    fprintf(stderr, "[burst] 発行 %d/%d (ok=%d fail=%d)\n", bi, g_nburst, bok, bfail);

    // (pt-loop は main iso loop 内でinline 実行、ここでの再ループは削除)

    // IT66121 SYS_STATUS 読み関数 (arm 前後比較用)
    // reg 0x0E は SYS_STATUS。ITE ドキュメント上 bit4=VID_STABLE
    #define IT66121_READ(reg) ({ \
        unsigned char _setup[3] = {0x9b, 0x01, (unsigned char)(reg)}; \
        libusb_control_transfer(h, 0x40, 0xC0, 0x5066, 0, _setup, 3, 1000); \
        unsigned char _resp[8]; \
        int _rr = libusb_control_transfer(h, 0xC0, 0xC0, 0x5066, 0, _resp, 1, 1000); \
        (_rr > 0) ? _resp[0] : 0xff; \
    })

    // arm 前の状態記録 (複数 register を i2c read - 各読みの間 5ms 空ける)
    #define IT66121_SNAP(tag) do { \
        unsigned char _r[16]; \
        for (int _i = 0; _i < 16; _i++) { \
            _r[_i] = IT66121_READ(_i); \
            usleep(2000); \
        } \
        fprintf(stderr, "[%s] IT66121 reg dump 0x00-0x0f:", tag); \
        for (int _i = 0; _i < 16; _i++) fprintf(stderr, " %02x", _r[_i]); \
        fprintf(stderr, "\n"); \
    } while (0)

    IT66121_SNAP("pre-arm");

    // Workflow synth 提案: MCU arm 60B バッチ を明示的に再送 (frame 13573 のペイロード)
    {
        static const unsigned char arm_payload[60] = {
            0x55, 0x80, 0x3c, 0x00, 0x6a, 0x80, 0x0f, 0x00, 0x6b, 0x80, 0xfe, 0x00,
            0x01, 0x81, 0x07, 0x00, 0x0b, 0x82, 0xdf, 0x00, 0x0c, 0x82, 0x3f, 0x00,
            0x0e, 0x82, 0x08, 0x00, 0x48, 0x82, 0x60, 0x00, 0x9b, 0x82, 0xf0, 0x00,
            0x11, 0x82, 0xff, 0x00, 0x12, 0x82, 0xff, 0x00, 0x11, 0x82, 0xff, 0x00,
            0x12, 0x82, 0xff, 0x00, 0x0e, 0x40, 0xd0, 0x9a, 0x0e, 0x40, 0x80, 0x9a
        };
        int r1 = libusb_control_transfer(h, 0x40, 0xC6, 0x0000, 0x0100, NULL, 0, 1000);
        int r2 = libusb_control_transfer(h, 0x40, 0xC6, 0x0032, 0x0101,
                                         (unsigned char*)arm_payload, 60, 1000);
        fprintf(stderr, "[arm] setup=%d arm=%d (60B MCU batch)\n", r1, r2);
    }
    usleep(50 * 1000);
    IT66121_SNAP("post-arm");

    // 追加試行: IT66121 ドライバ(Linux mainline) の "FireAFE + HDMI mode + unmute" 手順を明示送信
    // ite-it66121.c より:
    //   0x61 = 0x00 (FireAFE: AFE 起動)
    //   0xC0 = 0x01 (HDMI mode enable)
    //   0xC1 = 0x00 (AV unmute)
    //   0xC6 = 0x03 (packet generation)
    //   0x0F bit4 clear (TX clock)
    #define IT66121_WRITE(reg, val) do { \
        unsigned char _w[3] = {0x9a, (reg), (val)}; \
        libusb_control_transfer(h, 0x40, 0xC0, 0x5066, 0, _w, 3, 1000); \
    } while (0)

    fprintf(stderr, "[fire-afe] IT66121 output enable sequence...\n");
    IT66121_WRITE(0x61, 0x00); usleep(1000);   // FireAFE
    IT66121_WRITE(0xC0, 0x01); usleep(1000);   // HDMI mode
    IT66121_WRITE(0xC1, 0x00); usleep(1000);   // AV unmute
    IT66121_WRITE(0xC6, 0x03); usleep(1000);   // packet gen
    // 0x0F: 現状 0x00 なので bit4 は既に clear = TX clock 有効

    usleep(200 * 1000);
    IT66121_SNAP("post-fire");

    usleep(500 * 1000);
    IT66121_SNAP("+500ms   ");
    keep_running = 0;
    // 残りを回収
    struct timeval tv2 = {1, 0};
    for (int k = 0; k < 10 && inflight > 0; k++) libusb_handle_events_timeout(NULL, &tv2);

    if (outf) fclose(outf);
    if (g_v4l_fd >= 0) {
        enum v4l2_buf_type btype = V4L2_BUF_TYPE_VIDEO_OUTPUT;
        ioctl(g_v4l_fd, VIDIOC_STREAMOFF, &btype);
        close(g_v4l_fd);
    }
    audio_close();
    fprintf(stderr, "\n=== HD60S Linux driver stats ===\n");
    fprintf(stderr, "iso pkts:  ok=%ld empty=%ld err=%ld (empty=%.2f%%)\n",
            pkt_ok, pkt_empty, pkt_err,
            (pkt_ok + pkt_empty) ? 100.0 * pkt_empty / (pkt_ok + pkt_empty) : 0.0);
    fprintf(stderr, "iso total: %lld bytes / %d s = %.1f Mbps\n",
            total_bytes, read_sec, total_bytes * 8.0 / read_sec / 1e6);
    fprintf(stderr, "parser:    frames_emitted=%llu resyncs=%llu (empty=%llu marker=%llu overflow=%llu)\n",
            g_frames_out, g_resyncs, g_resync_empty, g_resync_marker, g_resync_overflow);
    fprintf(stderr, "audio:     frames=%llu underrun=%llu (%.2f s at 48kHz)\n",
            g_audio_frames, g_audio_underrun, g_audio_frames / 48000.0);

    libusb_release_interface(h, 0);
    libusb_close(h);
    libusb_exit(NULL);
    return 0;
}
