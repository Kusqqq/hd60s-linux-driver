// stream-iso.bin(生キャプチャ)からSEPペイロード(8B)を抜き出して、
// ・出現数を数える
// ・ペイロードだけをファイルに書き出す(音声候補)
// ラインパースは offline_parser.c と同じロジック。
//   build: gcc -O2 audio_extract.c -o audio_extract
//   run  : ./audio_extract captures/stream-iso.bin sep.raw [duration_sec_of_capture]
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

int main(int argc, char** argv) {
    if (argc < 3) { fprintf(stderr, "usage: %s input.bin sep.raw [dur_sec]\n", argv[0]); return 1; }
    double dur = argc >= 4 ? atof(argv[3]) : 0.0;
    FILE* f = fopen(argv[1], "rb"); if (!f) { perror(argv[1]); return 1; }
    fseek(f, 0, SEEK_END); size_t sz = ftell(f); fseek(f, 0, SEEK_SET);
    uint8_t* d = malloc(sz); fread(d, 1, sz, f); fclose(f);
    fprintf(stderr, "read %zu bytes\n", sz);

    FILE* out = fopen(argv[2], "wb"); if (!out) { perror(argv[2]); return 1; }

    uint64_t sep = 0, act = 0, blk = 0, unk = 0;
    uint64_t sep_bytes = 0;
    int lpos = 0;
    size_t i = 0;
    while (i + LINE_BYTES + 16 < sz) {
        size_t need = LINE_BYTES - lpos;
        i += need; lpos = LINE_BYTES;
        uint32_t tag; memcpy(&tag, d + i, 4);
        if (tag == MK_SEP) {
            fwrite(d + i + 4, 1, 8, out);
            sep++; sep_bytes += 8;
            i += 12;
            memcpy(&tag, d + i, 4);
        }
        i += 4;
        if (tag == MK_EOL_ACT) act++;
        else if (tag == MK_EOL_BLK) blk++;
        else unk++;
        lpos = 0;
    }
    fclose(out);

    fprintf(stderr, "SEP=%llu ACT=%llu BLK=%llu UNK=%llu\n",
        (unsigned long long)sep, (unsigned long long)act,
        (unsigned long long)blk, (unsigned long long)unk);
    fprintf(stderr, "SEP payload bytes: %llu (%.2f MB)\n",
        (unsigned long long)sep_bytes, sep_bytes / (1024.0*1024));

    if (dur > 0) {
        fprintf(stderr, "\n=== rate estimation over %.1f sec ===\n", dur);
        fprintf(stderr, "SEP rate: %.1f /sec\n", sep / dur);
        fprintf(stderr, "  if 8B = 1 stereo sample 16bit  -> %.1f Hz stereo\n", sep / dur);
        fprintf(stderr, "  if 8B = 2 stereo samples 16bit -> %.1f Hz stereo\n", 2 * sep / dur);
        fprintf(stderr, "  if 8B = 1 stereo sample 24bit  -> %.1f Hz stereo\n", sep / dur);
        fprintf(stderr, "  if 8B = IEC60958 subframe pair -> %.1f Hz stereo\n", sep / dur);
    } else {
        double frames = act / 1125.0;
        fprintf(stderr, "\nframes ~= %.1f (act/1125)\n", frames);
        fprintf(stderr, "SEP per frame: %.1f\n", (double)sep / frames);
        fprintf(stderr, "if 60fps -> SEP/sec = %.1f\n", 60.0 * sep / frames);
    }
    return 0;
}
