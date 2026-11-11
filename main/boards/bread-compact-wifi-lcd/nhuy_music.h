// ============================================================================
//  Nhu Y music player: plays MP3 files from the SD card (slot on the back of the
//  4.0" display, header J4) through the speaker, and exposes MCP tools so the AI
//  can list / play / stop songs by voice ("Như Ý ơi, phát bài Count on Me").
//
//  Copy .mp3 files to the root of the SD card or to a folder named "music".
//  The file name (without .mp3) is the song title.
// ============================================================================
#pragma once

#include "application.h"
#include "board.h"
#include "mcp_server.h"
#include "codecs/no_audio_codec.h"

#include <esp_log.h>
#include <esp_random.h>
#include <esp_heap_caps.h>
#include <esp_vfs_fat.h>
#include <driver/sdspi_host.h>
#include <driver/spi_common.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <dirent.h>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <string>
#include <vector>

#define MINIMP3_ONLY_MP3
#define MINIMP3_NO_SIMD
#define MINIMP3_IMPLEMENTATION
#include "minimp3.h"

// ---- SD card wiring (display header J4: SD_CS / SD_MOSI / SD_MISO / SD_SCK) ----
#define SD_CS_PIN    GPIO_NUM_10
#define SD_MOSI_PIN  GPIO_NUM_11
#define SD_SCK_PIN   GPIO_NUM_9
#define SD_MISO_PIN  GPIO_NUM_8
#ifndef SD_MOUNT
#define SD_MOUNT     "/sdcard"
#endif

// Codec wrapper: while music is playing the audio service must not power the
// speaker down (it does so after 15 s without its own output).
class NhuYCodec : public NoAudioCodecSimplex {
public:
    using NoAudioCodecSimplex::NoAudioCodecSimplex;
    static std::atomic<bool>& keep_output_on() {
        static std::atomic<bool> flag{false};
        return flag;
    }
    virtual void EnableOutput(bool enable) override {
        if (!enable && keep_output_on().load()) {
            return;
        }
        NoAudioCodecSimplex::EnableOutput(enable);
    }
};

class NhuYMusic {
public:
    static NhuYMusic& Get() {
        static NhuYMusic instance;
        return instance;
    }

    bool mounted() const { return mounted_; }
    bool playing() const { return playing_.load(); }
    bool busy() const { return playing_.load() || pending_.load(); }

    std::string title() {
        std::lock_guard<std::mutex> lock(mutex_);
        return title_;
    }

    // Mount the SD card, scan songs, start the player task and register MCP tools.
    void Initialize() {
        Mount();
        if (mounted_) {
            Scan();
        }
        queue_ = xQueueCreate(1, sizeof(int));
        StartTask();
        RegisterTools();
    }

    std::vector<std::string> Titles() {
        std::lock_guard<std::mutex> lock(mutex_);
        std::vector<std::string> out;
        for (auto& s : songs_) out.push_back(s.title);
        return out;
    }

    int Count() {
        std::lock_guard<std::mutex> lock(mutex_);
        return (int)songs_.size();
    }

    // interrupt = true: stop the current conversation right away (screen button).
    // interrupt = false: let Nhu Y finish her sentence first (voice request).
    bool PlayIndex(int index, bool interrupt) {
        if (index < 0 || index >= Count()) return false;
        stop_.store(true);  // stop a song that is already playing
        interrupt_.store(interrupt);
        pending_.store(true);
        xQueueOverwrite(queue_, &index);
        return true;
    }

    bool PlayRandom(bool interrupt) {
        int n = Count();
        if (n == 0) return false;
        int i = (int)(esp_random() % (uint32_t)n);
        if (n > 1 && i == last_index_) i = (i + 1) % n;
        return PlayIndex(i, interrupt);
    }

    // Best match by title; accents and case are ignored ("bai ca di hoc" = "Bài ca đi học").
    int Find(const std::string& query) {
        std::string q = Normalize(query);
        if (q.empty()) return -1;
        std::lock_guard<std::mutex> lock(mutex_);
        int best = -1, best_score = 0;
        for (int i = 0; i < (int)songs_.size(); i++) {
            const std::string& t = songs_[i].key;
            int score = 0;
            if (t == q) score = 1000;
            else if (t.find(q) != std::string::npos) score = 500 + (int)q.size();
            else if (q.find(t) != std::string::npos) score = 400 + (int)t.size();
            else score = CommonWords(QueryWords(query), songs_[i].title) * 10;
            if (score > best_score) {
                best_score = score;
                best = i;
            }
        }
        return best;
    }

    void Stop() {
        pending_.store(false);
        stop_.store(true);
    }

private:
    struct Song {
        std::string path;
        std::string title;
        std::string key;  // normalized title for matching
    };

    bool mounted_ = false;
    std::mutex mutex_;
    std::vector<Song> songs_;
    std::string title_;
    QueueHandle_t queue_ = nullptr;
    std::atomic<bool> playing_{false};
    std::atomic<bool> pending_{false};
    std::atomic<bool> stop_{false};
    std::atomic<bool> interrupt_{false};
    int last_index_ = -1;

    /* ---------------- SD card ---------------- */
    void Mount() {
        spi_bus_config_t bus = {};
        bus.mosi_io_num = SD_MOSI_PIN;
        bus.miso_io_num = SD_MISO_PIN;
        bus.sclk_io_num = SD_SCK_PIN;
        bus.quadwp_io_num = -1;
        bus.quadhd_io_num = -1;
        bus.max_transfer_sz = 16 * 1024;
        esp_err_t err = spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO);
        if (err != ESP_OK) {
            ESP_LOGE("NhuYMusic", "SD SPI bus init failed: %s", esp_err_to_name(err));
            return;
        }
        static sdspi_device_config_t slot = {};
        slot.host_id = SPI2_HOST;
        slot.gpio_cs = SD_CS_PIN;
        slot.gpio_cd = SDSPI_SLOT_NO_CD;
        slot.gpio_wp = GPIO_NUM_NC;
        slot.gpio_int = GPIO_NUM_NC;

        esp_vfs_fat_sdmmc_mount_config_t mount = {};
        mount.format_if_mount_failed = false;
        mount.max_files = 3;
        mount.allocation_unit_size = 16 * 1024;

        sdmmc_host_t host = SDSPI_HOST_DEFAULT();
        host.slot = SPI2_HOST;
        sdmmc_card_t* card = nullptr;
        err = esp_vfs_fat_sdspi_mount(SD_MOUNT, &host, &slot, &mount, &card);
        if (err != ESP_OK) {
            ESP_LOGW("NhuYMusic", "No SD card (%s)", esp_err_to_name(err));
            return;
        }
        mounted_ = true;
        ESP_LOGI("NhuYMusic", "SD card mounted");
    }

    static bool IsMp3(const char* name) {
        size_t n = strlen(name);
        if (n < 5 || name[0] == '.') return false;
        const char* e = name + n - 4;
        return (e[0] == '.') && (e[1] == 'm' || e[1] == 'M') && (e[2] == 'p' || e[2] == 'P') &&
               (e[3] == '3');
    }

    void ScanDir(const std::string& dir) {
        DIR* d = opendir(dir.c_str());
        if (d == nullptr) return;
        struct dirent* ent;
        while ((ent = readdir(d)) != nullptr && songs_.size() < 400) {
            if (!IsMp3(ent->d_name)) continue;
            Song s;
            s.path = dir + "/" + ent->d_name;
            s.title = std::string(ent->d_name, strlen(ent->d_name) - 4);
            s.key = Normalize(s.title);
            songs_.push_back(s);
        }
        closedir(d);
    }

    void Scan() {
        std::lock_guard<std::mutex> lock(mutex_);
        songs_.clear();
        ScanDir(SD_MOUNT);
        ScanDir(SD_MOUNT "/music");
        ScanDir(SD_MOUNT "/MUSIC");
        ESP_LOGI("NhuYMusic", "%d songs on SD card", (int)songs_.size());
    }

    /* ---------------- text matching ---------------- */
    // Vietnamese / Latin accents -> plain lowercase ASCII letters.
    static char BaseLetter(uint32_t cp) {
        static const char* const latin =  // U+00C0 .. U+01B0
            "aaaaaa?ceeeeiiii?nooooo??uuuuy??aaaaaa?ceeeeiiii?nooooo??uuuuy?yaaaaaaccccccccdd"
            "ddeeeeeeeeeegggggggghh??iiiiiiiii???jjkk?llllll????nnnnnn???oooooo??rrrrrrssssss"
            "sstttt??uuuuuuuuuuuuwwyyyzzzzzz?????????????????????????????????oo?????????????u"
            "u";
        static const char* const viet =  // U+1EA0 .. U+1EF9
            "aaaaaaaaaaaaaaaaaaaaaaaaeeeeeeeeeeeeeeeeiiiioooooooooooooooooooooooouuuuuuuuuuuu"
            "uuyyyyyyyy";
        if (cp < 0x80) {
            char c = (char)cp;
            if (c >= 'A' && c <= 'Z') c = (char)(c - 'A' + 'a');
            return ((c >= 'a' && c <= 'z') || (c >= '0' && c <= '9')) ? c : ' ';
        }
        if (cp == 0x111 || cp == 0x110) return 'd';
        if (cp >= 0xC0 && cp <= 0x1B0) return latin[cp - 0xC0] == '?' ? ' ' : latin[cp - 0xC0];
        if (cp >= 0x1EA0 && cp <= 0x1EF9) return viet[cp - 0x1EA0];
        return ' ';
    }

    static std::string Words(const std::string& s) {
        std::string out;
        const unsigned char* p = (const unsigned char*)s.c_str();
        while (*p) {
            uint32_t cp;
            int len = 1;
            if (*p < 0x80) cp = *p;
            else if ((*p & 0xE0) == 0xC0 && p[1]) { cp = ((p[0] & 0x1F) << 6) | (p[1] & 0x3F); len = 2; }
            else if ((*p & 0xF0) == 0xE0 && p[1] && p[2]) {
                cp = ((p[0] & 0x0F) << 12) | ((p[1] & 0x3F) << 6) | (p[2] & 0x3F);
                len = 3;
            } else { cp = ' '; len = 1; }
            char c = BaseLetter(cp);
            if (c != ' ' || (!out.empty() && out.back() != ' ')) out += c;
            p += len;
        }
        while (!out.empty() && out.back() == ' ') out.pop_back();
        return out;
    }

    static std::string Normalize(const std::string& s) {
        std::string w = Words(s), out;
        for (char c : w) if (c != ' ') out += c;
        return out;
    }

    static std::vector<std::string> QueryWords(const std::string& s) {
        std::vector<std::string> v;
        std::string w = Words(s), cur;
        for (char c : w) {
            if (c == ' ') { if (cur.size() > 1) v.push_back(cur); cur.clear(); }
            else cur += c;
        }
        if (cur.size() > 1) v.push_back(cur);
        return v;
    }

    static int CommonWords(const std::vector<std::string>& q, const std::string& title) {
        std::string t = " " + Words(title) + " ";
        int n = 0;
        for (auto& w : q) if (t.find(" " + w + " ") != std::string::npos) n++;
        return n;
    }

    /* ---------------- MCP tools ---------------- */
    void RegisterTools() {
        auto& mcp = McpServer::GetInstance();
        mcp.AddTool("self.music.list",
                    "List the songs stored on this device's SD card (English and Vietnamese "
                    "songs). Call this to know which songs can be played with self.music.play.",
                    PropertyList(), [this](const PropertyList&) -> ReturnValue {
                        std::string out = "{\"songs\":[";
                        auto titles = Titles();
                        for (size_t i = 0; i < titles.size(); i++) {
                            if (i) out += ",";
                            out += "\"" + JsonEscape(titles[i]) + "\"";
                        }
                        out += "],\"sd_card\":";
                        out += mounted_ ? "true}" : "false}";
                        return out;
                    });
        mcp.AddTool("self.music.play",
                    "Play a song from this device's SD card. ALWAYS use this tool first when "
                    "the user asks to play / listen to a song or music (English or Vietnamese). "
                    "`name` is the song title, accents are optional. The song starts right "
                    "after you finish your short reply, so keep the reply to one sentence.",
                    PropertyList({Property("name", kPropertyTypeString)}),
                    [this](const PropertyList& p) -> ReturnValue {
                        int i = Find(p["name"].value<std::string>());
                        if (i < 0) {
                            return std::string("{\"ok\":false,\"error\":\"song not found on SD "
                                               "card, call self.music.list\"}");
                        }
                        PlayIndex(i, false);
                        return "{\"ok\":true,\"playing\":\"" + JsonEscape(TitleAt(i)) + "\"}";
                    });
        mcp.AddTool("self.music.play_random",
                    "Play a random song from this device's SD card.", PropertyList(),
                    [this](const PropertyList&) -> ReturnValue {
                        if (!PlayRandom(false)) {
                            return std::string("{\"ok\":false,\"error\":\"no songs\"}");
                        }
                        return std::string("{\"ok\":true}");
                    });
        mcp.AddTool("self.music.stop", "Stop the song that is playing from the SD card.",
                    PropertyList(), [this](const PropertyList&) -> ReturnValue {
                        Stop();
                        return true;
                    });
    }

    std::string TitleAt(int i) {
        std::lock_guard<std::mutex> lock(mutex_);
        return (i >= 0 && i < (int)songs_.size()) ? songs_[i].title : "";
    }

    static std::string JsonEscape(const std::string& s) {
        std::string o;
        for (char c : s) {
            if (c == '"' || c == '\\') { o += '\\'; o += c; }
            else if ((unsigned char)c >= 0x20) o += c;
        }
        return o;
    }

    /* ---------------- player task ---------------- */
    void StartTask() {
        const uint32_t stack = 32 * 1024;  // minimp3 keeps ~16 KB of scratch on the stack
        BaseType_t ok = xTaskCreatePinnedToCoreWithCaps(
            [](void* arg) { static_cast<NhuYMusic*>(arg)->TaskLoop(); }, "nhuy_music", stack, this,
            3, nullptr, 1, MALLOC_CAP_SPIRAM);
        if (ok != pdPASS) {
            xTaskCreatePinnedToCore([](void* arg) { static_cast<NhuYMusic*>(arg)->TaskLoop(); },
                                    "nhuy_music", stack, this, 3, nullptr, 1);
        }
    }

    // Wait until the conversation is over (or stop it), so the song does not mix with speech.
    bool WaitForIdle() {
        auto& app = Application::GetInstance();
        bool aborted = false;
        for (int t = 0; t < 300; t++) {  // up to 30 s
            if (!pending_.load()) return false;
            DeviceState s = app.GetDeviceState();
            if (s == kDeviceStateIdle) return true;
            if (s == kDeviceStateSpeaking && interrupt_.load() && !aborted) {
                app.Schedule([]() { Application::GetInstance().AbortSpeaking(kAbortReasonNone); });
                aborted = true;
            } else if (s == kDeviceStateListening && (t % 10) == 0) {
                app.ToggleChatState();  // closes the audio channel -> idle
            }
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        return false;
    }

    void TaskLoop() {
        int index = 0;
        while (true) {
            if (xQueueReceive(queue_, &index, portMAX_DELAY) != pdTRUE) continue;
            stop_.store(false);
            if (!WaitForIdle()) {
                pending_.store(false);
                continue;
            }
            Song song;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (index < 0 || index >= (int)songs_.size()) { pending_.store(false); continue; }
                song = songs_[index];
                title_ = song.title;
            }
            last_index_ = index;
            pending_.store(false);
            PlayFile(song.path);
            std::lock_guard<std::mutex> lock(mutex_);
            title_.clear();
        }
    }

    void PlayFile(const std::string& path) {
        auto codec = Board::GetInstance().GetAudioCodec();
        FILE* f = fopen(path.c_str(), "rb");
        if (f == nullptr || codec == nullptr) {
            ESP_LOGE("NhuYMusic", "Cannot open %s", path.c_str());
            if (f) fclose(f);
            return;
        }
        const size_t kBuf = 16 * 1024;
        auto* buf = (uint8_t*)heap_caps_malloc(kBuf, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        auto* dec = (mp3dec_t*)heap_caps_malloc(sizeof(mp3dec_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        auto* pcm = (int16_t*)heap_caps_malloc(MINIMP3_MAX_SAMPLES_PER_FRAME * sizeof(int16_t),
                                                MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (buf == nullptr || dec == nullptr || pcm == nullptr) {
            ESP_LOGE("NhuYMusic", "Out of memory");
            free(buf); free(dec); free(pcm); fclose(f);
            return;
        }
        mp3dec_init(dec);
        ESP_LOGI("NhuYMusic", "Playing %s", path.c_str());

        NhuYCodec::keep_output_on().store(true);
        playing_.store(true);
        codec->EnableOutput(true);

        const int out_rate = codec->output_sample_rate();
        std::vector<int16_t> out;
        out.reserve(2048);
        float pos = 0.0f;   // resampler position (relative to current frame, -1 = last sample)
        int16_t last = 0;
        size_t filled = 0;
        bool eof = false;
        auto& app = Application::GetInstance();

        while (!stop_.load()) {
            if (app.GetDeviceState() != kDeviceStateIdle) break;  // user started talking
            if (!eof && filled < kBuf / 2) {
                size_t r = fread(buf + filled, 1, kBuf - filled, f);
                if (r == 0) eof = true;
                filled += r;
            }
            if (filled == 0) break;
            mp3dec_frame_info_t info;
            int samples = mp3dec_decode_frame(dec, buf, (int)filled, pcm, &info);
            if (info.frame_bytes == 0) {
                if (eof) break;
                filled = 0;  // garbage: drop buffer and read more
                continue;
            }
            filled -= info.frame_bytes;
            memmove(buf, buf + info.frame_bytes, filled);
            if (samples <= 0 || info.hz <= 0) continue;

            // Stereo -> mono
            if (info.channels == 2) {
                for (int i = 0; i < samples; i++) {
                    pcm[i] = (int16_t)(((int)pcm[2 * i] + (int)pcm[2 * i + 1]) / 2);
                }
            }
            // Linear resampling to the speaker rate
            const float step = (float)info.hz / (float)out_rate;
            out.clear();
            while (pos < (float)(samples - 1)) {
                int i = (int)floorf(pos);
                float fr = pos - (float)i;
                int a = (i < 0) ? last : pcm[i];
                int b = pcm[i + 1];
                out.push_back((int16_t)(a + (b - a) * fr));
                pos += step;
            }
            pos -= (float)samples;
            last = pcm[samples - 1];
            if (!out.empty()) {
                codec->OutputData(out);
            }
        }

        playing_.store(false);
        NhuYCodec::keep_output_on().store(false);
        free(buf);
        free(dec);
        free(pcm);
        fclose(f);
        ESP_LOGI("NhuYMusic", "Stopped");
    }
};
