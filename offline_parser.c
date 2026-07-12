// stream-iso.bin(生キャプチャ)を読み込み、ラインマーカーパースで
// 4147200Bのフレームを取り出して先頭Nフレームを PNG に(実際はraw YUYV に出力)する
// 検証用ミニツール。iso_capture.c のパーサと同一ロジック。
//   build: gcc -O2 offline_parser.c -o offline_parser
//   run  : ./offline_parser captures/stream-iso.bin captures/proof/frame_%d.yuv
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#define FRAME_W 1920
#define FRAME_H 1080
#define LINE_BYTES (FRAME_W * 2)
#define FRAME_BYTES (LINE_BYTES * FRAME_H)

#define MK_EOL_ACT 0x800000ffu
#define MK_EOL_BLK 0xab0000ffu
#define MK_SEP     0x02ff00ffu

static uint8_t framebuf[FRAME_BYTES];
static int fline = 0;
static int blk_run = 0;
static int locked = 0;
static uint64_t frames = 0;
static uint64_t resyncs = 0;
static const char* out_pat;
static int max_out = 3;

static void emit(void) {
    frames++;
    if (frames <= (uint64_t)max_out) {
        char path[512]; snprintf(path, sizeof(path), out_pat, (int)frames);
        FILE* f = fopen(path, "wb");
        if (f) { fwrite(framebuf, 1, FRAME_BYTES, f); fclose(f); fprintf(stderr, "wrote %s\n", path); }
    }
}

static void reset(const char* why) {
    if (locked) fprintf(stderr, "resync: %s fline=%d\n", why, fline);
    locked = 0; fline = 0; blk_run = 0;
    resyncs++;
}

int main(int argc, char** argv) {
    if (argc < 3) { fprintf(stderr, "usage: %s input.bin output_%%d.yuv\n", argv[0]); return 1; }
    out_pat = argv[2];
    FILE* f = fopen(argv[1], "rb"); if (!f) { perror(argv[1]); return 1; }
    fseek(f, 0, SEEK_END); size_t sz = ftell(f); fseek(f, 0, SEEK_SET);
    uint8_t* d = malloc(sz); fread(d, 1, sz, f); fclose(f);
    fprintf(stderr, "read %zu bytes\n", sz);

    static uint8_t linebuf[LINE_BYTES];
    int lpos = 0;
    size_t i = 0;
    while (i + LINE_BYTES + 16 < sz) {
        // 3840B詰める
        size_t need = LINE_BYTES - lpos;
        memcpy(linebuf + lpos, d + i, need);
        i += need; lpos = LINE_BYTES;
        // マーカー4B(SEP なら 16B)
        uint32_t tag; memcpy(&tag, d + i, 4);
        if (tag == MK_SEP) { i += 12; memcpy(&tag, d + i, 4); }
        i += 4;
        if (tag == MK_EOL_ACT) {
            if (!locked) {
                if (blk_run >= 30) { locked = 1; fline = 0; }
                else { lpos = 0; continue; }
            }
            if (blk_run > 0 && fline >= FRAME_H) { emit(); fline = 0; }
            blk_run = 0;
            if (fline < FRAME_H) memcpy(framebuf + fline * LINE_BYTES, linebuf, LINE_BYTES);
            fline++;
        } else if (tag == MK_EOL_BLK) {
            blk_run++;
        } else {
            reset("unknown");
        }
        lpos = 0;
    }
    fprintf(stderr, "frames=%llu resyncs=%llu\n",
            (unsigned long long)frames, (unsigned long long)resyncs);
    return 0;
}
