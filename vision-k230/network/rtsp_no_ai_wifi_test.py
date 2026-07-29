import network  # 缃戠粶妯″潡锛岀敤浜庡鐞?WiFi 杩炴帴 / Network module for handling WiFi connections
import os      # 鎿嶄綔绯荤粺鎺ュ彛妯″潡 / Operating system interface module
import time    # 鏃堕棿妯″潡锛岀敤浜庡欢鏃舵搷浣?/ Time module for delay operations
import _thread # 绾跨▼妯″潡锛岀敤浜庡绾跨▼鎿嶄綔 / Thread module for multithreading operations
import gc      # 鍨冨溇鍥炴敹妯″潡锛岀敤浜庡唴瀛樼鐞?/ Garbage collection module for memory management
import sys     # 绯荤粺妯″潡锛岀敤浜庣郴缁熺浉鍏虫搷浣?/ System module for system-related operations
import random  # 闅忔満鏁版ā鍧?/ Random number module
import ujson   # JSON 澶勭悊妯″潡 / JSON processing module
import utime   # 寰绾ф椂闂存ā鍧?/ Microsecond-level time module
import ulab.numpy as np  # 鏁板€艰绠楀簱 / Numerical computation library
import nncase_runtime as nn  # 绁炵粡缃戠粶杩愯鏃跺簱 / Neural network runtime library
import aidemo  # AI 婕旂ず妯″潡 / AI demo module
import image   # 鍥惧儚澶勭悊妯″潡 / Image processing module
import multimedia as mm  # 澶氬獟浣撴ā鍧?/ Multimedia module
from time import sleep  # 浠?time 妯″潡瀵煎叆 sleep 鍑芥暟 / Import sleep function from time module
from media.vencoder import *  # 浠庡獟浣撴ā鍧楀鍏ヨ棰戠紪鐮佸櫒鐩稿叧鍔熻兘 / Import video encoder-related functions from media module
from media.sensor import *    # 浠庡獟浣撴ā鍧楀鍏ヤ紶鎰熷櫒鐩稿叧鍔熻兘 / Import sensor-related functions from media module
from media.media import *     # 浠庡獟浣撴ā鍧楀鍏ュ獟浣撶鐞嗗姛鑳?/ Import media management functions from media module
from media.display import *   # 浠庡獟浣撴ā鍧楀鍏ユ樉绀虹浉鍏冲姛鑳?/ Import display-related functions from media module
from libs.PipeLine import PipeLine, ScopedTiming  # 浠?libs 瀵煎叆 PipeLine 鍜?ScopedTiming 绫?/ Import PipeLine and ScopedTiming classes from libs
from libs.AIBase import AIBase  # 浠?libs 瀵煎叆 AIBase 绫?/ Import AIBase class from libs
from libs.AI2D import Ai2d      # 浠?libs 瀵煎叆 Ai2d 绫?/ Import Ai2d class from libs

# Connect to WiFi
# 杩炴帴鍒?WiFi 缃戠粶
def Connect_WIFI(ID, PASSWORD):
    sta = network.WLAN(0)  # 鍒涘缓 WLAN 瀵硅薄锛? 琛ㄧず绔欐ā寮?/ Create WLAN object, 0 indicates station mode
    if sta.isconnected():  # 妫€鏌ユ槸鍚﹀凡杩炴帴 / Check if already connected
        sta.disconnect()   # 濡傛灉宸茶繛鎺ワ紝鍒欐柇寮€杩炴帴 / Disconnect if already connected
        time.sleep(1)      # 绛夊緟 1 绉?/ Wait for 1 second

    sta.connect(ID, PASSWORD)  # 杩炴帴鍒版寚瀹氱殑 WiFi 缃戠粶 / Connect to the specified WiFi network
    # 鏌ョ湅鏄惁杩炴帴鎴愬姛 / Check if connection is successful
    while sta.ifconfig()[0] == '0.0.0.0':  # 濡傛灉 IP 鍦板潃涓?'0.0.0.0'锛岃〃绀烘湭杩炴帴 / If IP address is '0.0.0.0', it means not connected
        time.sleep(1)                      # 姣忕妫€鏌ヤ竴娆?/ Check every second

    print(sta.ifconfig()[0])  # 鎵撳嵃鑾峰彇鍒扮殑 IP 鍦板潃 / Print the obtained IP address

    return sta.isconnected()  # 杩斿洖杩炴帴鐘舵€?/ Return connection status

# RTSP Server class
# RTSP 鏈嶅姟鍣ㄧ被
class RtspServer:
    def __init__(self, session_name="video", port=8554, video_type=mm.multi_media_type.media_h264,
                 enable_audio=False, sensor=None, initMediaManager=False):
        self.session_name = session_name  # 浼氳瘽鍚嶇О / Session name
        self.video_type = video_type      # 瑙嗛绫诲瀷锛欻.264/H.265 / Video type: H.264/H.265
        self.enable_audio = enable_audio  # 鏄惁鍚敤闊抽 / Whether to enable audio
        self.port = port                  # RTSP 绔彛鍙?/ RTSP port number
        self.rtspserver = mm.rtsp_server()  # 瀹炰緥鍖?RTSP 鏈嶅姟鍣?/ Instantiate RTSP server
        self.venc_chn = VENC_CHN_ID_0     # VENC 閫氶亾 / VENC channel
        self.start_stream = False         # 鏄惁鍚姩鎺ㄦ祦绾跨▼ / Whether to start the streaming thread
        self.runthread_over = False       # 鎺ㄦ祦绾跨▼鏄惁宸茬粨鏉?/ Whether the streaming thread has finished
        self.sensor = sensor              # 浼犳劅鍣ㄥ璞?/ Sensor object
        self.initMediaManager = initMediaManager  # 鏄惁鍒濆鍖栧獟浣撶鐞嗗櫒 / Whether to initialize media manager

    # Start the RTSP server
    # 鍚姩 RTSP 鏈嶅姟鍣?
    def start(self):
        # 鍒濆鍖栨帹娴?/ Initialize stream
        self._init_stream()
        self.rtspserver.rtspserver_init(self.port)  # 鍒濆鍖?RTSP 鏈嶅姟鍣紝鎸囧畾绔彛 / Initialize RTSP server with specified port
        # 鍒涘缓浼氳瘽 / Create session
        self.rtspserver.rtspserver_createsession(self.session_name, self.video_type, self.enable_audio)
        # 鍚姩 RTSP 鏈嶅姟鍣?/ Start RTSP server
        self.rtspserver.rtspserver_start()
        self._start_stream()  # 鍚姩鎺ㄦ祦 / Start streaming

        # 鍚姩鎺ㄦ祦绾跨▼ / Start streaming thread
        self.start_stream = True
        _thread.start_new_thread(self._do_rtsp_stream, ())  # 鍒涘缓鏂扮嚎绋嬭繍琛屾帹娴佸嚱鏁?/ Create a new thread to run the streaming function

    # Stop the RTSP server
    # 鍋滄 RTSP 鏈嶅姟鍣?
    def stop(self):
        if self.start_stream == False:  # 濡傛灉鎺ㄦ祦鏈惎鍔紝鐩存帴杩斿洖 / If streaming hasn鈥檛 started, return directly
            return
        # 绛夊緟鎺ㄦ祦绾跨▼閫€鍑?/ Wait for the streaming thread to exit
        self.start_stream = False
        while not self.runthread_over:  # 寰幆绛夊緟绾跨▼缁撴潫 / Loop until the thread ends
            sleep(0.1)                 # 姣?0.1 绉掓鏌ヤ竴娆?/ Check every 0.1 seconds
        self.runthread_over = False    # 閲嶇疆绾跨▼缁撴潫鏍囧織 / Reset thread completion flag

        # 鍋滄鎺ㄦ祦 / Stop streaming
        self._stop_stream()
        self.rtspserver.rtspserver_stop()  # 鍋滄 RTSP 鏈嶅姟鍣?/ Stop RTSP server
        # self.rtspserver.rtspserver_destroysession(self.session_name)  # 閿€姣佷細璇濓紙宸叉敞閲婏級 / Destroy session (commented out)
        self.rtspserver.rtspserver_deinit()  # 鍙嶅垵濮嬪寲 RTSP 鏈嶅姟鍣?/ Deinitialize RTSP server

    # Get the RTSP URL
    # 鑾峰彇 RTSP 鍦板潃
    def get_rtsp_url(self):
        return self.rtspserver.rtspserver_getrtspurl(self.session_name)  # 杩斿洖 RTSP 鍦板潃 / Return RTSP URL

    # Initialize the stream
    # 鍒濆鍖栨帹娴?
    def _init_stream(self):
        # 璁剧疆瑙嗛鍒嗚鲸鐜囷紙浠ヤ笅涓哄彲閫夊垎杈ㄧ巼锛屽凡娉ㄩ噴閮ㄥ垎涓哄叾浠栭€夐」锛?/ Set video resolution (commented sections are other options)
        # width = 1280
        # height = 720
        # width = 640
        # height = 360
        # width = 1920
        # height = 1080
        width = 512   # 褰撳墠瀹藉害 / Current width
        height = 288  # 褰撳墠楂樺害 / Current height
        # width = 384
        # height = 216

        width = ALIGN_UP(width, 16)  # 灏嗗搴﹀榻愬埌 16 鐨勫€嶆暟 / Align width to a multiple of 16
        # 鍒濆鍖栦紶鎰熷櫒 / Initialize sensor
        self.sensor = Sensor()       # 鍒涘缓浼犳劅鍣ㄥ璞?/ Create sensor object
        self.sensor.reset()          # 閲嶇疆浼犳劅鍣?/ Reset sensor
        self.sensor.set_framesize(width=width, height=height, alignment=12)  # 璁剧疆甯уぇ灏?/ Set frame size
        self.sensor.set_pixformat(Sensor.YUV420SP)  # 璁剧疆鍍忕礌鏍煎紡涓?YUV420SP / Set pixel format to YUV420SP

        # 瀹炰緥鍖栬棰戠紪鐮佸櫒 / Instantiate video encoder
        self.encoder = Encoder()     # 鍒涘缓缂栫爜鍣ㄥ璞?/ Create encoder object
        self.encoder.SetOutBufs(self.venc_chn, 8, width, height)  # 璁剧疆杈撳嚭缂撳啿鍖?/ Set output buffers
        # 缁戝畾鐩告満鍜?VENC锛堝凡娉ㄩ噴锛屽綋鍓嶆湭浣跨敤锛?/ Bind camera and VENC (commented out, not currently used)
        # self.link = MediaManager.link(self.sensor.bind_info()['src'], (VIDEO_ENCODE_MOD_ID, VENC_DEV_ID, self.venc_chn))

        self.link = None  # 鍒濆鍖栭摼鎺ヤ负 None / Initialize link as None
        # 鍒濆鍖栧獟浣撶鐞嗗櫒 / Initialize media manager
        MediaManager.init()  # 璋冪敤濯掍綋绠＄悊鍣ㄧ殑鍒濆鍖栧嚱鏁?/ Call the media manager鈥檚 initialization function
        # 鍒涘缓缂栫爜鍣?/ Create encoder
        chnAttr = ChnAttrStr(
            self.encoder.PAYLOAD_TYPE_H264,
            self.encoder.H264_PROFILE_MAIN,
            width,
            height,
            bit_rate=2000,
            dst_frame_rate=15,
            src_frame_rate=15
        )
        # 璁剧疆缂栫爜鍣ㄥ睘鎬э細H.264 绫诲瀷锛屼富閰嶇疆鏂囦欢锛屽搴︼紝楂樺害 / Set encoder attributes: H.264 type, main profile, width, height
        self.encoder.Create(self.venc_chn, chnAttr)  # 鍒涘缓缂栫爜鍣ㄩ€氶亾 / Create encoder channel

    # Start the stream
    # 鍚姩鎺ㄦ祦
    def _start_stream(self):
        # 寮€濮嬬紪鐮?/ Start encoding
        self.encoder.Start(self.venc_chn)  # 鍚姩缂栫爜鍣ㄩ€氶亾 / Start encoder channel
        # 鍚姩鐩告満 / Start camera
        self.sensor.run()  # 杩愯浼犳劅鍣?/ Run sensor

    # Stop the stream
    # 鍋滄鎺ㄦ祦
    def _stop_stream(self):
        # 鍋滄鐩告満 / Stop camera
        self.sensor.stop()  # 鍋滄浼犳劅鍣?/ Stop sensor
        # 瑙ｇ粦鐩告満鍜?VENC / Unbind camera and VENC
        del self.link       # 鍒犻櫎閾炬帴瀵硅薄 / Delete link object
        # 鍋滄缂栫爜 / Stop encoding
        self.encoder.Stop(self.venc_chn)     # 鍋滄缂栫爜鍣ㄩ€氶亾 / Stop encoder channel
        self.encoder.Destroy(self.venc_chn)  # 閿€姣佺紪鐮佸櫒閫氶亾 / Destroy encoder channel
        # 娓呯悊缂撳啿鍖?/ Clear buffer
        MediaManager.deinit()  # 鍙嶅垵濮嬪寲濯掍綋绠＄悊鍣?/ Deinitialize media manager

    # RTSP streaming thread
    # RTSP 鎺ㄦ祦绾跨▼
    def _do_rtsp_stream(self):
        try:
            streamData = StreamData()  # 鍒涘缓娴佹暟鎹璞?/ Create stream data object
            frame_info = k_video_frame_info()  # 鍒涘缓瑙嗛甯т俊鎭璞?/ Create video frame info object

            while self.start_stream:  # 褰撴帹娴佹爣蹇椾负 True 鏃跺惊鐜?/ Loop while streaming flag is True
                # 鎹曡幏涓€甯?/ Capture a frame
                rtsp_show_img = self.sensor.snapshot()  # 浠庝紶鎰熷櫒鑾峰彇涓€甯у浘鍍?/ Get one frame from sensor

                if rtsp_show_img == -1:  # 濡傛灉鎹曡幏澶辫触锛岃烦杩囨湰娆″惊鐜?/ If capture fails, skip this iteration
                    continue
                frame_info.v_frame.width = rtsp_show_img.width()    # 璁剧疆甯у搴?/ Set frame width
                frame_info.v_frame.height = rtsp_show_img.height()  # 璁剧疆甯ч珮搴?/ Set frame height
                frame_info.v_frame.pixel_format = Sensor.YUV420SP   # 璁剧疆鍍忕礌鏍煎紡 / Set pixel format
                frame_info.pool_id = rtsp_show_img.poolid()      # 璁剧疆缂撳啿姹?ID / Set buffer pool ID
                frame_info.v_frame.phys_addr[0] = rtsp_show_img.phyaddr()  # 璁剧疆绗竴骞抽潰鐨勭墿鐞嗗湴鍧€ / Set physical address of the first plane

                # 鏍规嵁鍥惧儚澶у皬璁剧疆绗簩骞抽潰鐨勭墿鐞嗗湴鍧€ / Set the physical address of the second plane based on image size
                if rtsp_show_img.width() == 800 and rtsp_show_img.height() == 480:
                    frame_info.v_frame.phys_addr[1] = frame_info.v_frame.phys_addr[0] + \
                        frame_info.v_frame.width * frame_info.v_frame.height + 1024
                elif rtsp_show_img.width() == 1920 and rtsp_show_img.height() == 1080:
                    frame_info.v_frame.phys_addr[1] = frame_info.v_frame.phys_addr[0] + \
                        frame_info.v_frame.width * frame_info.v_frame.height + 3072
                elif rtsp_show_img.width() == 640 and rtsp_show_img.height() == 360:
                    frame_info.v_frame.phys_addr[1] = frame_info.v_frame.phys_addr[0] + \
                        frame_info.v_frame.width * frame_info.v_frame.height + 3072
                else:
                    frame_info.v_frame.phys_addr[1] = frame_info.v_frame.phys_addr[0] + \
                        frame_info.v_frame.width * frame_info.v_frame.height

                # 灏嗗抚鍙戦€佸埌缂栫爜鍣?/ Send the frame to the encoder
                self.encoder.SendFrame(self.venc_chn, frame_info)
                self.encoder.GetStream(self.venc_chn, streamData)  # 鑾峰彇涓€甯ф祦鏁版嵁 / Get a frame of stream data

                # 灏嗙紪鐮佹暟鎹彂閫佸埌 RTSP 鏈嶅姟鍣?/ Send encoded data to the RTSP server
                for pack_idx in range(0, streamData.pack_cnt):  # 閬嶅巻鏁版嵁鍖?/ Iterate over data packets
                    stream_data = bytes(uctypes.bytearray_at(streamData.data[pack_idx], streamData.data_size[pack_idx]))
                    # 灏嗘暟鎹浆鎹负瀛楄妭娴?/ Convert data to byte stream
                    # print("stream size: ", streamData.data_size[pack_idx], "stream type: ", streamData.stream_type[pack_idx])
                    self.rtspserver.rtspserver_sendvideodata(self.session_name, stream_data,
                                                             streamData.data_size[pack_idx], 1000)
                    # 鍙戦€佽棰戞暟鎹埌 RTSP 鏈嶅姟鍣?/ Send video data to RTSP server

                self.encoder.ReleaseStream(self.venc_chn, streamData)  # 閲婃斁涓€甯ф祦鏁版嵁 / Release a frame of stream data

                ######################################

                gc.collect()         # 鍨冨溇鍥炴敹锛岄噴鏀惧唴瀛?/ Garbage collection to free memory
                time.sleep_us(10)    # 寤舵椂 10 寰 / Delay for 10 microseconds
                os.exitpoint()       # 妫€鏌ラ€€鍑虹偣 / Check exit point

        except BaseException as e:
            print(f"Exception {e}")  # 鎹曡幏骞舵墦鍗板紓甯?/ Catch and print exceptions
        finally:
            self.runthread_over = True  # 璁剧疆绾跨▼缁撴潫鏍囧織 / Set thread completion flag
            # 鍋滄 RTSP 鏈嶅姟鍣?/ Stop RTSP server
            self.stop()

        self.runthread_over = True  # 纭繚绾跨▼缁撴潫鏍囧織涓?True / Ensure thread completion flag is True

if __name__ == "__main__":
    print("[WIFI] 杩炴帴缃戠粶涓?Connecting to network ...")  # 鎻愮ず姝ｅ湪杩炴帴缃戠粶 / Indicate network connection in progress
    # 杩炴帴 WiFi 缃戠粶 / Connect to WiFi network
    isConnected = Connect_WIFI("YOUR_SSID", "YOUR_PASSWORD")  # 浣跨敤鎸囧畾 ID 鍜屽瘑鐮佽繛鎺?/ Connect using specified ID and password
    if isConnected:
        print("[WIFI] 缃戠粶杩炴帴鎴愬姛 Network connection successful")  # 杩炴帴鎴愬姛鎻愮ず / Connection successful message
    else:
        import sys
        print("[WIFI] 缃戠粶杩炴帴澶辫触 Network connection failed! Please check the configuration")
        # 杩炴帴澶辫触鎻愮ず / Connection failed message
        time.sleep_ms(10)  # 寤舵椂 10 姣 / Delay for 10 milliseconds
        sys.exit()         # 閫€鍑虹▼搴?/ Exit program

    print("[RTSP] Starting ...")  # 鎻愮ず RTSP 鍚姩 / Indicate RTSP starting
    time.sleep(1)                # 寤舵椂 1 绉?/ Delay for 1 second

    # 鍒涘缓 RTSP 鏈嶅姟鍣ㄥ璞?/ Create RTSP server object
    rtspserver = RtspServer()
    # 鍚姩 RTSP 鏈嶅姟鍣?/ Start RTSP server
    rtspserver.start()
    # 鎵撳嵃 RTSP 鍦板潃 / Print RTSP URL
    rtsp_address = rtspserver.get_rtsp_url()
    print("[RTSP] Started successfully, address:", rtsp_address)  # 鍚姩鎴愬姛骞舵樉绀哄湴鍧€ / Started successfully and show address

    # 鎺ㄦ祦 60 绉?/ Stream for 60 seconds
    while True:
        time.sleep_ms(10)  # 姣?10 姣寰幆涓€娆?/ Loop every 10 milliseconds
    # 鍋滄 RTSP 鏈嶅姟鍣?/ Stop RTSP server
    rtspserver.stop()
    print("done")  # 鎻愮ず瀹屾垚 / Indicate completion
