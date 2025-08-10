# 電子手振れ補正＆水平維持 ver. 1.2
# 1.0：ボタンのへこみ解消検討、1.1:ffmpeg、1.2：30フレームちょうどで終了した場合の不具合、csvのy、z軸交換、ffmpeg GUIconsole qiuet
import tkinter as tk
from tkinter import ttk
import os
import tkinter.filedialog
import ffmpeg
import pandas as x_datas
import time
import numpy as np
import quaternion
import cv2
from scipy.interpolate import CubicSpline
import datetime
import math
import glob
import winsound
import kalman_def_1
import sys


##import subprocess

class GuiSet:
    def __init__(self, root):

        # massage表示
        self.message = tk.Message(root, width=450, text="")
        self.message.config(font=('メイリオ', 12))
        self.message.place(x=20, y=680)

        # textbox表示
        self.textbox = tk.Entry(width=10)
        self.textbox.config(font=('メイリオ', 12))
        self.textbox.place(x=20, y=620)

        # progressbarの作成、配置
        self.progress_ttk = ttk.Progressbar(root, length=300)
        self.progress_ttk.place(x=200, y=620)

        # 動画ファイル選択のボタン
        self.button = tk.Button(root, text='動画ファイルを選択', font=('メイリオ', 12),
                                width=30, height=1, bg='#999999', activebackground="#aaaaaa")
        # self.button.bind('<ButtonPress>', self.file_dialog)
        self.button.bind('<ButtonPress>', self.on_click)  ######################################
        self.button.place(x=20, y=20)

        # 選択したファイル名のラベル
        self.file_name = tk.StringVar()
        self.path_file = tk.StringVar()  # 追加
        self.file_name.set('未選択です')
        self.path_file.set('')  # 追加
        self.label = tk.Label(textvariable=self.file_name, font=('メイリオ', 12), justify=tk.RIGHT)
        self.label.place(x=20, y=80)

        # 処理開始ボタン
        self.button1 = tk.Button(root, text="処理開始", font=('メイリオ', 12), width=18,
                                 height=1, bg='#999999', activebackground="#aaaaaa")
        self.button1.bind('<ButtonPress>', self.button_click)
        self.button1.place(x=20, y=560)

        # 選択値格納用変数
        self.var0 = tk.IntVar(root)
        self.var0.set(1)  # プログラム起動時の初期値
        self.var1 = tk.IntVar(root)
        self.var1.set(2)  # プログラム起動時の初期値
        self.var2 = tk.IntVar(root)
        self.var2.set(2)  # プログラム起動時の初期値

        # Radiobutton
        self.frame1 = tk.LabelFrame(root, text="水平維持軸の選択")
        self.frame1.place(x=20, y=140)
        self.rb01 = tk.Radiobutton(self.frame1, text="XY平面", value=1, var=self.var0, font=('メイリオ', 12),
                                   command=self.grav_kalman)
        self.rb01.pack(pady=5)
        self.rb02 = tk.Radiobutton(self.frame1, text="roll軸", value=2, var=self.var0, font=('メイリオ', 12),
                                   command=self.grav_kalman)
        self.rb02.pack(pady=5)
        self.rb03 = tk.Radiobutton(self.frame1, text="なし", value=3, var=self.var0, font=('メイリオ', 12),
                                   command=self.grav_kalman)
        self.rb03.pack(pady=5)

        self.frame2 = tk.LabelFrame(root, text="四元数の計算")
        self.frame2.place(x=180, y=140)
        self.rb11 = tk.Radiobutton(self.frame2, text="CORI", value=1, var=self.var1, font=('メイリオ', 12),
                                   command=self.delta_entry)
        self.rb11.pack(pady=5)
        self.rb12 = tk.Radiobutton(self.frame2, text="GYRO", value=2, var=self.var1, font=('メイリオ', 12),
                                   command=self.delta_entry)
        self.rb12.pack(pady=5)

        self.frame3 = tk.LabelFrame(root, text="重力ベクトルの計算")
        self.frame3.place(x=320, y=140)
        self.rb21 = tk.Radiobutton(self.frame3, text="GRAV", value=1, var=self.var2, font=('メイリオ', 12))
        self.rb21.pack(pady=5)
        self.rb22 = tk.Radiobutton(self.frame3, text="Kalman", value=2, var=self.var2, font=('メイリオ', 12))
        self.rb22.pack(pady=5)

        # δラベル
        self.label_delta = tk.Label(root, text='GYRO同期調整 δ（FPS時間単位）', font=('メイリオ', 12))
        self.label_delta.place(x=20, y=320)

        # δ入力
        self.delta_txt = tk.Entry(root, width=10, font=('メイリオ', 12))
        self.delta_txt.place(x=300, y=320)
        self.delta_txt.delete(0, tk.END)  # tkinter.END
        self.delta_txt.insert(0, '0.52')

        # pitch軸オフセットラベル
        self.label_pitch = tk.Label(root, text='pitch軸のオフセット（度）（＋側が下向き）', font=('メイリオ', 12))
        self.label_pitch.place(x=20, y=400)

        # pitch軸オフセットスケール
        self.scale1 = tk.Scale(root, font=('メイリオ', 12), orient="vertical", from_=30, to=-30)
        self.scale1.place(x=360, y=360)

        # 映像のリニア―／広角ラベル
        self.label_pitch = tk.Label(root, text='出力映像：リニア（0.0）←→ 広角（1.0）', font=('メイリオ', 12))
        self.label_pitch.place(x=20, y=490)

        # 映像のリニア／広角スケール
        self.scale2 = tk.Scale(root, font=('メイリオ', 12), orient="horizontal", from_=0, to=1, resolution=0.1)
        self.scale2.place(x=360, y=480)

        # 終了ボタン
        self.button_exit = tk.Button(root, text="画面を閉じる", font=('メイリオ', 12), width=18, height=1, bg='#999999',
                                     activebackground="#aaaaaa", command=self.exit_program)
        self.button_exit.place(x=300, y=560)

    # CORI or GYRO選択によるδ入力の不可
    def delta_entry(self):
        if self.var1.get() == 1:
            self.delta_txt.config(state=tk.DISABLED)
        else:
            self.delta_txt.config(state=tk.NORMAL)

    # GRAV/Kalman選択の有効・無効化
    def grav_kalman(self):
        if self.var0.get() == 3:
            self.rb21.config(state=tk.DISABLED)
            self.rb22.config(state=tk.DISABLED)
        else:
            self.rb21.config(state=tk.NORMAL)
            self.rb22.config(state=tk.NORMAL)

    # 処理開始ボタンクリック時処理
    def button_click(self, event):

        self.button1.config(state='disabled')
        self.button1["text"] = "処理中"
        self.message.config(text="電子手振れ補正＆水平維持の四元数を計算しています。しばらくお待ちください。")
        select0 = self.var0.get()
        select1 = self.var1.get()
        select2 = self.var2.get()
        if select0 == 1:
            horizon_select = 0
        elif select0 == 2:
            horizon_select = 1
        else:
            horizon_select = 4
        if select1 == 1:
            quaternion_select = 1
        else:
            quaternion_select = 2
        if select2 == 1:
            grav_n = 0
        else:
            grav_n = 3
        file_name_2 = self.file_name.get()
        if file_name_2 == "未選択です":
            print('ファイル未選択')
            self.file_name.set('未選択です')
            self.button1.config(state='normal')
            self.button1["text"] = "処理開始"
            self.message.config(text=" ")
            return
        path_file_2 = self.path_file.get()
        # GoPro 5.3K 30FPS 動画判定
        video_info = ffmpeg.probe(path_file_2)
        is_GoPro = False
        if 'streams' in video_info:
            if len(video_info['streams']) == 4:
                if 'width' in video_info['streams'][0] and 'height' in video_info['streams'][0] and 'r_frame_rate' in \
                        video_info['streams'][0]:
                    width = video_info['streams'][0]['width']
                    height = video_info['streams'][0]['height']
                    r_frame_rate = video_info['streams'][0]['r_frame_rate']
                    if width == 5312 and height == 4648 and r_frame_rate == '30000/1001':
                        if 'codec_tag_string' in video_info['streams'][3]:
                            codec_tag_string = video_info['streams'][3]['codec_tag_string']
                            if codec_tag_string == 'gpmd':
                                is_GoPro = True
                                print(codec_tag_string,'found by ffprobe')
                                print('this is Gopro 5.3K 30FPS video', is_GoPro)
        if not is_GoPro:
            print('this is not GoPro 5.3K 30FPS video')
            self.file_name.set('GoPro 5.3K 30FPS 映像ではありません')
            self.button1.config(state='normal')
            self.button1["text"] = "処理開始"
            self.button["text"] = "動画ファイルを選択"
            self.message.config(text=" ")
            return
        timestamp_shift = self.delta_txt.get()
        pitch_offset = self.scale1.get()
        dist_parameter = self.scale2.get()
        if select0 == 1:
            print('水平維持の選択：xy平面')
        elif select0 == 2:
            print('水平維持の選択：roll軸')
        else:
            print('水平維持の選択：なし')
        if select1 == 1:
            print('四元数の計算：CORI')
        else:
            print('四元数の計算：GYRO')
        if select2 == 1:
            print('重力ベクトルの計算：GRAV')
        else:
            print('重力ベクトルの計算：Kalman')
        print('file name: ', file_name_2)  # >> GX010785.MP4
        print('path file: ', path_file_2)  # 追加 >> C:/Users/dalan/PycharmProjects/PythonProject/.venv/GX010785.MP4
        print('delta: ', timestamp_shift)
        print('pitch offset: ', pitch_offset)
        print('distortion: ', dist_parameter)
        root.update()
        file_name_2 = file_name_2[:-4]  # 拡張子省略 # GX010785
        path_file_2 = path_file_2[:-4]  # 拡張子省略 # C:/Users/dalan/PycharmProjects/PythonProject/.venv/GX010785

        # mp3音声ファイル抽出
        ffmpeg.input(path_file_2 + '.mp4').output(file_name_2 + '.mp3', vn=None).run(capture_stdout=True,
                                                                                     capture_stderr=True,
                                                                                     overwrite_output=True, quiet=True)
        print('mp3 audio extract done')

        # metadataを抽出し、binファイルを生成
        ffmpeg.input(path_file_2 + '.mp4').output(file_name_2 + '.bin', codec='copy', map='0:3', format='rawvideo').run(
            overwrite_output=True, quiet=True)
        print('bin extract done')

        # 四元数の計算
        cori_crop_1, dist_para = kalman_def_1.stab_horizon(file_name_2, path_file_2, horizon_select, quaternion_select,
                                                           grav_n, timestamp_shift, pitch_offset, dist_parameter)
        print('cori_crop_1: ', len(cori_crop_1))
        total_count = len(cori_crop_1)
        # 映像変換
        counter = kalman_def_1.stabilization_53_87of18K_tkinter(file_name_2, path_file_2, cori_crop_1, dist_para)
        for count in counter:
            if count == 1:
                start = time.time()
                # print('start time: ',start)
                self.message.config(text=f"フレーム総数： {total_count} フレーム毎に映像変換処理を行っています。")
            if count == 50:
                current = time.time()
                time_per_frame = "{:.2f}".format((current - start) / count)
                self.message.config(
                    text=f"フレーム総数： {total_count}  映像１フレーム当たりの処理時間（秒）(最初の50フレーム平均)： {time_per_frame}")
                print(f"映像１フレーム当たりの処理時間（秒）＜最初の50フレーム平均＞：{time_per_frame}")
                time_per_frame_1 = time_per_frame
            if count == total_count - 1:
                current = time.time()
                time_per_frame = "{:.2f}".format((current - start) / count)
                self.message.config(
                    text=f"映像変換が完了。フレーム総数： {total_count}  映像１フレーム当たりの処理時間（秒）＜最初の50フレーム平均＞： {time_per_frame_1} 、＜全フレーム平均＞：　{time_per_frame}")
                print(f"映像１フレーム当たりの処理時間（秒）＜全フレーム平均＞：{time_per_frame}")
                winsound.Beep(700, 500)
            progress = (count + 1) / total_count * 100
            current = time.time()
            SecToConvert = current - start
            self.progress_ttk['value'] = progress
            self.textbox.delete(0, tk.END)
            time_diff = str(datetime.timedelta(seconds=SecToConvert))
            self.textbox.insert(0, time_diff[:8])
            root.update()
            end = time.time()
        # 音声のファイルmp3の付加
        input_video = file_name_2 + '_stabilized.mp4'
        input_audio = file_name_2 + '.mp3'
        output_video = file_name_2 + '_stabilized_with_audio.mp4'
        bitrate = '180M'  # 100Mbps
        ffmpeg.concat(ffmpeg.input(input_video), ffmpeg.input(input_audio), v=1, a=1).output(output_video,
                                                                                             video_bitrate=bitrate).run(overwrite_output=True, quiet=True)
        print(' ')
        print('audio file concat')
        print('all completed')
        self.textbox.delete(0, tk.END)
        self.button1["text"] = "すべて完了"
        root.update()
        SecToConvert = end - start
        time_diff = str(datetime.timedelta(seconds=SecToConvert))
        self.textbox.insert(0, time_diff[:8])

        winsound.Beep(700, 500)

    # def file_dialog(self, event):
    def file_dialog(self):  ###################################################################
        fTyp = [("", "*.mp4")]
        iDir = os.path.abspath(os.path.dirname(__name__))
        path_file_1 = tk.filedialog.askopenfilename(filetypes=fTyp, initialdir=iDir)
        if len(path_file_1) == 0:
            self.file_name.set('選択をキャンセルしました')
        else:
            file_name_1 = os.path.basename(path_file_1)
            self.file_name.set(file_name_1)
            self.path_file.set(path_file_1)

        self.button["text"] = "下記の動画ファイルが選択されました"
        root.update()

    # afterを使って処理を1ミリ秒遅らせる button のへこみ解消
    def on_click(self, event) -> None:  #####################################
        print('come to on_click')
        root.after(1, self.file_dialog)  # bad argument "0.5": must be cancel, idle, info, or an integer

    def exit_program(self):
        root.quit()
        exit()


if __name__ == "__main__":
    path = os.getcwd()
    print('current path: ', path)
    root = tk.Tk()
    root.title("電子手振れ補正＆水平維持 ver. 1.2")
    root.geometry("520x800")
    ins = GuiSet(root)
    root.mainloop()