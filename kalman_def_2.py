# colab専用 kalman_def_2
import numpy as np
import quaternion
import cv2
#import ffmpeg # for colab
from scipy.interpolate import CubicSpline
import datetime
import math
import glob
from matplotlib import pyplot as plt
import matplotlib.pyplot as plt
import pandas as x_datas
import time
import os
import sys
import kalman_def_2 # for colab
import subprocess

# def list
#
# bin_extract_4    :　bin fileからGYRO（ジャイロデータ）等を読み替える（TimeWarp動画のケース）
# bin_extract_5    :　bin fileからGYRO（ジャイロデータ）等を読み替える（RealSpeed動画のケース）
# kalman_filter_63baxis_final_beta_xyz_1 : 6軸Kalmam Filterから重力ベクトルを計算
# stab_horizon     : 電子手振れ補正＆水平維持の逆四元数を計算
# horizon          : 水平維持の四元数を計算
# stabilization_53_87of18K_tkinter : 映像変換（広角 ⇒ リニア ⇒ 透視変換 ⇒ 広角)
# add_black_border : 映像変換する際、映像処理領域（画素数）を拡大
# calc_H6bxyz, calc_h6b, calc_f6bxyz, calc_F6bxyz, calc_f_beta : Kalman係数を計算するための行列計算
# gc_lpf_half_ele_var, gc_lpf : 四元数のガウス畳み込み法による平滑化計算
# gc_lpf_element    : １次元要素のガウス畳み込み法による平滑化計算
#

# timewarp
#def bin_extract_4(file_name):# gyro_data,accl_data　x,y順番もとに戻す
def bin_extract_4(path_file):
    # バイナリファイルのパス(binary file path)
    #file_bin = file_name+'.bin'
    file_bin = path_file +'.bin'# "/content/drive/MyDrive/GX010785"+".bin"
    # バイナリファイルを読み込む(read binary file)
    data = np.fromfile(file_bin, dtype=np.uint8) # unsign int8
    data_1 = np.fromfile(file_bin, dtype=np.int8) # sign int8
    # NumPy配列を2次元の形状に変換する(transform into 2 dimension shape)
    data = data.reshape((-1, 2))
    data_1=data_1.reshape((-1,2))

    fourcc_times=0

    a_0 = np.zeros([1, 1], dtype='int64')
    a_1 = np.zeros([1, 1], dtype='int64')
    b_0 = np.zeros([1, 1], dtype='int64')
    b_1 = np.zeros([1, 1], dtype='int64')
    c_0 = np.zeros([1, 1], dtype='int64')
    c_1 = np.zeros([1, 1], dtype='int64')
    d_0 = np.zeros([1, 1], dtype='int64')
    d_1 = np.zeros([1, 1], dtype='int64')
    e_0 = np.zeros([1, 1], dtype='int64')
    e_1 = np.zeros([1, 1], dtype='int64')
    f_0 = np.zeros([1, 1], dtype='int64')
    f_1 = np.zeros([1, 1], dtype='int64')
    g_0 = np.zeros([1, 1], dtype='int64')
    g_1 = np.zeros([1, 1], dtype='int64')
    h_0 = np.zeros([1, 1], dtype='int64')
    h_1 = np.zeros([1, 1], dtype='int64')
    i_0 = np.zeros([1, 1], dtype='int64')
    i_1 = np.zeros([1, 1], dtype='int64')
    timestamp = np.zeros([1, 1], dtype='uint64')
    timestamp_1 = np.zeros([1, 1], dtype='uint64')
    timestamp_2 = np.zeros([1, 1], dtype='uint64')
    timestamp_3 = np.zeros([1, 1], dtype='uint64')
    timestamp_4 = np.zeros([1, 1], dtype='uint64')
    prev_timestamp = np.zeros([1, 1], dtype='uint64')
    prev_timestamp_1 = np.zeros([1, 1], dtype='uint64')
    prev_timestamp_2 = np.zeros([1, 1], dtype='uint64')
    prev_timestamp_3 = np.zeros([1, 1], dtype='uint64')
    prev_timestamp_4 = np.zeros([1, 1], dtype='uint64')
    scale = np.zeros([1, 1], dtype='int64')
    scale_1 = np.zeros([1, 1], dtype='int64')
    scale_2 = np.zeros([1, 1], dtype='int64')
    scale_3 = np.zeros([1, 1], dtype='int64')
    scale_4 = np.zeros([1, 1], dtype='int64')

    for i in range(data.shape[0]-1):
        FourCC=chr(data[i,0])+chr(data[i,1])
        FourCC+=chr(data[i+1,0])+chr(data[i+1,1])
        if FourCC=='ACCL':
            fourcc_times+=1

    gyro_data=np.zeros([fourcc_times*31,4])
    accl_data=np.zeros([fourcc_times*31,4])
    cori_data=np.zeros([fourcc_times*31,5])
    grav_data=np.zeros([fourcc_times*31,4])
    iori_data=np.zeros([fourcc_times*31,5])
    unix_data=np.zeros([accl_size])

    flag=0
    flag_1=0
    gyro_count=0
    gyro_count_1=0
    accl_count=0
    accl_count_1=0
    flag_2=0
    cori_count=0
    cori_count_1=0
    flag_3=0
    grav_count=0
    grav_count_1=0
    flag_4=0
    iori_count=0
    iori_count_1=0
    unix_count=0
    diff_timestamp=0
    diff_timestamp_1=0
    diff_timestamp_2=0
    diff_timestamp_3=0
    diff_timestamp_4=0
    gpsu_times=0
    accl_times=0
    gyro_times=0
    cori_times=0
    grav_times=0
    iori_times=0

    for i in range(data.shape[0]-1):

        FourCC=chr(data[i,0])+chr(data[i,1])
        FourCC+=chr(data[i+1,0])+chr(data[i+1,1])

        if FourCC=='GPSU':# no=GPS GoPro12
            gpsu_times+=1
            year=int('20'+chr(data[i+4,0])+chr(data[i+4,1]))
            month=int(chr(data[i+5,0])+chr(data[i+5,1]))
            day=int(chr(data[i+6,0])+chr(data[i+6,1]))
            hour=int(chr(data[i+7,0])+chr(data[i+7,1]))
            minute=int(chr(data[i+8,0])+chr(data[i+8,1]))
            sec=int(chr(data[i+9,0])+chr(data[i+9,1]))
            microsec=int(chr(data[i+10,1])+chr(data[i+11,0])+chr(data[i+11,0]))*1000
            unix_time=datetime.datetime.timestamp(datetime.datetime(year, month, day, hour, minute, sec, microsec))
            unix_data[unix_count] = unix_time+9*60*60
            unix_count+=1

        if FourCC=='ACCL':
            accl_times+=1
            #timestamp=data[i-46,0]*256**7+data[i-46,1]*256**6+data[i-45,0]*256**5+data[i-45,1]*256**4
            #timestamp+=data[i-44,0]*256**3+data[i-44,1]*256**2+data[i-43,0]*256+data[i-43,1]
            a_0[0,0]=data[i-46,0]
            a_1[0,0]=data[i-46,1]
            b_0[0,0]=data[i-45,0]
            b_1[0,0]=data[i-45,1]
            c_0[0,0]=data[i-44,0]
            c_1[0,0]=data[i-44,1]
            d_0[0,0]=data[i-43,0]
            d_1[0,0]=data[i-43,1]
            timestamp[0,0]=a_0[0,0]*256**7+a_1[0,0]*256**6+b_0[0,0]*256**5+b_1[0,0]*256**4+c_0[0,0]*256**3+c_1[0,0]*256**2+d_0[0,0]*256+d_1[0,0]
            repeat=data[i+3,1]
            #scale=data[i-8,0]*256+data[i-8,1]
            e_0[0,0]=data[i-8,0]
            e_1[0,0]=data[i-8,1]
            scale[0,0]=e_0[0,0]*256+e_1[0,0]

            for j in range(repeat):
                f_0[0,0]=data_1[i+4+j*3,0]
                f_1[0,0]=data[i+4+j*3,1]
                g_0[0,0]=data_1[i+5+j*3,0]
                g_1[0,0]=data[i+5+j*3,1]
                h_0[0,0]=data_1[i+6+j*3,0]
                h_1[0,0]=data[i+6+j*3,1]
                #x_accl=(data_1[i+4+j*3,0]*256+data[i+4+j*3,1])/scale[0,0]
                #y_accl=(data_1[i+5+j*3,0]*256+data[i+5+j*3,1])/scale[0,0]
                #z_accl=(data_1[i+6+j*3,0]*256+data[i+6+j*3,1])/scale[0,0]
                x_accl=(f_0[0,0]*256+f_1[0,0])/scale[0,0]
                y_accl=(g_0[0,0]*256+g_1[0,0])/scale[0,0]
                z_accl=(h_0[0,0]*256+h_1[0,0])/scale[0,0]
                accl_data[accl_count,3]=x_accl#軸入替、定義に準ずる
                accl_data[accl_count,1]=y_accl
                accl_data[accl_count,2]=z_accl
                accl_count+=1

            if flag==0:
                prev_repeat=repeat
                prev_timestamp[0,0]=timestamp[0,0]

            if flag!=0:
                diff_timestamp=(timestamp[0,0]-prev_timestamp[0,0])/prev_repeat
                for j in range(prev_repeat):
                    accl_data[accl_count_1,0]=prev_timestamp[0,0]+diff_timestamp*j
                    accl_count_1+=1

            prev_repeat=repeat
            prev_timestamp[0,0]=timestamp[0,0]
            flag=1

        if FourCC=='GYRO':
            gyro_times+=1
            #timestamp_1=data[i-46,0]*256**7+data[i-46,1]*256**6+data[i-45,0]*256**5+data[i-45,1]*256**4
            #timestamp_1+=data[i-44,0]*256**3+data[i-44,1]*256**2+data[i-43,0]*256+data[i-43,1]
            a_0[0,0]=data[i-46,0]
            a_1[0,0]=data[i-46,1]
            b_0[0,0]=data[i-45,0]
            b_1[0,0]=data[i-45,1]
            c_0[0,0]=data[i-44,0]
            c_1[0,0]=data[i-44,1]
            d_0[0,0]=data[i-43,0]
            d_1[0,0]=data[i-43,1]
            timestamp_1[0,0]=a_0[0,0]*256**7+a_1[0,0]*256**6+b_0[0,0]*256**5+b_1[0,0]*256**4+c_0[0,0]*256**3+c_1[0,0]*256**2+d_0[0,0]*256+d_1[0,0]
            repeat_1=data[i+3,1]
            #scale_1=data[i-8,0]*256+data[i-8,1]
            e_0[0,0]=data[i-8,0]
            e_1[0,0]=data[i-8,1]
            scale_1[0,0]=e_0[0,0]*256+e_1[0,0]

            for j in range(repeat_1):
                f_0[0,0]=data_1[i+4+j*3,0]
                f_1[0,0]=data[i+4+j*3,1]
                g_0[0,0]=data_1[i+5+j*3,0]
                g_1[0,0]=data[i+5+j*3,1]
                h_0[0,0]=data_1[i+6+j*3,0]
                h_1[0,0]=data[i+6+j*3,1]
                #x_gyro=(data_1[i+4+j*3,0]*256+data[i+4+j*3,1])/scale_1[0,0]
                #y_gyro=(data_1[i+5+j*3,0]*256+data[i+5+j*3,1])/scale_1[0,0]
                #z_gyro=(data_1[i+6+j*3,0]*256+data[i+6+j*3,1])/scale_1[0,0]
                x_gyro=(f_0[0,0]*256+f_1[0,0])/scale[0,0]
                y_gyro=(g_0[0,0]*256+g_1[0,0])/scale[0,0]
                z_gyro=(h_0[0,0]*256+h_1[0,0])/scale[0,0]
                gyro_data[gyro_count,3]=x_gyro#軸入替、定義に準ずる
                gyro_data[gyro_count,1]=y_gyro
                gyro_data[gyro_count,2]=z_gyro
                gyro_count+=1

            if flag_1==0:
                prev_repeat_1=repeat_1
                prev_timestamp_1[0,0]=timestamp_1[0,0]

            if flag_1!=0:
                diff_timestamp_1=(timestamp_1[0,0]-prev_timestamp_1[0,0])/prev_repeat_1
                for j in range(prev_repeat_1):
                    gyro_data[gyro_count_1,0]=prev_timestamp_1[0,0]+diff_timestamp_1*j
                    gyro_count_1+=1

            prev_repeat_1=repeat_1
            prev_timestamp_1[0,0]=timestamp_1[0,0]
            flag_1=1

        if FourCC=='CORI':
            cori_times+=1
            #timestamp_2=data[i-30,0]*256**7+data[i-30,1]*256**6+data[i-29,0]*256**5+data[i-29,1]*256**4
            #timestamp_2+=data[i-28,0]*256**3+data[i-28,1]*256**2+data[i-27,0]*256+data[i-27,1]
            a_0[0,0]=data[i-30,0]
            a_1[0,0]=data[i-30,1]
            b_0[0,0]=data[i-29,0]
            b_1[0,0]=data[i-29,1]
            c_0[0,0]=data[i-28,0]
            c_1[0,0]=data[i-28,1]
            d_0[0,0]=data[i-27,0]
            d_1[0,0]=data[i-27,1]
            timestamp_2[0,0]=a_0[0,0]*256**7+a_1[0,0]*256**6+b_0[0,0]*256**5+b_1[0,0]*256**4+c_0[0,0]*256**3+c_1[0,0]*256**2+d_0[0,0]*256+d_1[0,0]
            repeat_2=data[i+3,1]
            #scale_2=data[i-2,0]*256+data[i-2,1]
            e_0[0,0]=data[i-2,0]
            e_1[0,0]=data[i-2,1]
            scale_2[0,0]=e_0[0,0]*256+e_1[0,0]

            for j in range(repeat_2):
                f_0[0,0]=data_1[i+4+j*4,0]
                f_1[0,0]=data[i+4+j*4,1]
                g_0[0,0]=data_1[i+5+j*4,0]
                g_1[0,0]=data[i+5+j*4,1]
                h_0[0,0]=data_1[i+6+j*4,0]
                h_1[0,0]=data[i+6+j*4,1]
                i_0[0,0]=data_1[i+7+j*4,0]
                i_1[0,0]=data[i+7+j*4,1]
                #w_cori=(data_1[i+4+j*4,0]*256+data[i+4+j*4,1])/scale_2[0,0]
                #x_cori=(data_1[i+5+j*4,0]*256+data[i+5+j*4,1])/scale_2[0,0]
                #y_cori=(data_1[i+6+j*4,0]*256+data[i+6+j*4,1])/scale_2[0,0]
                #z_cori=(data_1[i+7+j*4,0]*256+data[i+7+j*4,1])/scale_2[0,0]
                w_cori=(f_0[0,0]*256+f_1[0,0])/scale_2[0,0]
                x_cori=(g_0[0,0]*256+g_1[0,0])/scale_2[0,0]
                y_cori=(h_0[0,0]*256+h_1[0,0])/scale_2[0,0]
                z_cori=(i_0[0,0]*256+i_1[0,0])/scale_2[0,0]

                cori_data[cori_count,1]=w_cori
                cori_data[cori_count,2]=x_cori
                cori_data[cori_count,4]=y_cori #GoPro firmwareのバグ？？　y,z軸入替
                cori_data[cori_count,3]=z_cori
                cori_count+=1

            if flag_2==0:
                prev_repeat_2=repeat_2
                prev_timestamp_2[0,0]=timestamp_2[0,0]

            if flag_2!=0:
                diff_timestamp_2=(timestamp_2[0,0]-prev_timestamp_2[0,0])/prev_repeat_2
                for j in range(prev_repeat_2):
                    cori_data[cori_count_1,0]=prev_timestamp_2[0,0]+diff_timestamp_2*j
                    cori_count_1+=1

            prev_repeat_2=repeat_2
            prev_timestamp_2[0,0]=timestamp_2[0,0]
            flag_2=1

        if FourCC=='GRAV':
            grav_times+=1
            #timestamp_3=data[i-28,0]*256**7+data[i-28,1]*256**6+data[i-27,0]*256**5+data[i-27,1]*256**4
            #timestamp_3+=data[i-26,0]*256**3+data[i-26,1]*256**2+data[i-25,0]*256+data[i-25,1]
            a_0[0,0]=data[i-28,0]
            a_1[0,0]=data[i-28,1]
            b_0[0,0]=data[i-27,0]
            b_1[0,0]=data[i-27,1]
            c_0[0,0]=data[i-26,0]
            c_1[0,0]=data[i-26,1]
            d_0[0,0]=data[i-25,0]
            d_1[0,0]=data[i-25,1]
            timestamp_3[0,0]=a_0[0,0]*256**7+a_1[0,0]*256**6+b_0[0,0]*256**5+b_1[0,0]*256**4+c_0[0,0]*256**3+c_1[0,0]*256**2+d_0[0,0]*256+d_1[0,0]
            repeat_3=data[i+3,1]
            #scale_3=data[i-2,0]*256+data[i-2,1]
            e_0[0,0]=data[i-2,0]
            e_1[0,0]=data[i-2,1]
            scale_3[0,0]=e_0[0,0]*256+e_1[0,0]

            for j in range(repeat_3):
                f_0[0,0]=data_1[i+4+j*3,0]
                f_1[0,0]=data[i+4+j*3,1]
                g_0[0,0]=data_1[i+5+j*3,0]
                g_1[0,0]=data[i+5+j*3,1]
                h_0[0,0]=data_1[i+6+j*3,0]
                h_1[0,0]=data[i+6+j*3,1]

                #x_grav=(data_1[i+4+j*3,0]*256+data[i+4+j*3,1])/scale_3[0,0]
                #y_grav=(data_1[i+5+j*3,0]*256+data[i+5+j*3,1])/scale_3[0,0]
                #z_grav=(data_1[i+6+j*3,0]*256+data[i+6+j*3,1])/scale_3[0,0]
                x_grav=(f_0[0,0]*256+f_1[0,0])/scale_3[0,0]
                y_grav=(g_0[0,0]*256+g_1[0,0])/scale_3[0,0]
                z_grav=(h_0[0,0]*256+h_1[0,0])/scale_3[0,0]

                grav_data[grav_count,1]=x_grav
                grav_data[grav_count,2]=y_grav
                grav_data[grav_count,3]=z_grav

                grav_count+=1

            if flag_3==0:
                prev_repeat_3=repeat_3
                prev_timestamp_3[0,0]=timestamp_3[0,0]

            if flag_3!=0:
                diff_timestamp_3=(timestamp_3[0,0]-prev_timestamp_3[0,0])/prev_repeat_3
                for j in range(prev_repeat_3):
                    grav_data[grav_count_1,0]=prev_timestamp_3[0,0]+diff_timestamp_3*j
                    grav_count_1+=1

            prev_repeat_3=repeat_3
            prev_timestamp_3[0,0]=timestamp_3[0,0]
            flag_3=1

        if FourCC=='IORI':
            iori_times+=1
            #timestamp_4=data[i-28,0]*256**7+data[i-28,1]*256**6+data[i-27,0]*256**5+data[i-27,1]*256**4
            #timestamp_4+=data[i-26,0]*256**3+data[i-26,1]*256**2+data[i-25,0]*256+data[i-25,1]
            a_0[0,0]=data[i-28,0]
            a_1[0,0]=data[i-28,1]
            b_0[0,0]=data[i-27,0]
            b_1[0,0]=data[i-27,1]
            c_0[0,0]=data[i-26,0]
            c_1[0,0]=data[i-26,1]
            d_0[0,0]=data[i-25,0]
            d_1[0,0]=data[i-25,1]
            timestamp_4[0,0]=a_0[0,0]*256**7+a_1[0,0]*256**6+b_0[0,0]*256**5+b_1[0,0]*256**4+c_0[0,0]*256**3+c_1[0,0]*256**2+d_0[0,0]*256+d_1[0,0]
            repeat_4=data[i+3,1]
            #scale_4=data[i-2,0]*256+data[i-2,1]
            e_0[0,0]=data[i-2,0]
            e_1[0,0]=data[i-2,1]
            scale_4[0,0]=e_0[0,0]*256+e_1[0,0]

            for j in range(repeat_4):
                f_0[0,0]=data_1[i+4+j*4,0]
                f_1[0,0]=data[i+4+j*4,1]
                g_0[0,0]=data_1[i+5+j*4,0]
                g_1[0,0]=data[i+5+j*4,1]
                h_0[0,0]=data_1[i+6+j*4,0]
                h_1[0,0]=data[i+6+j*4,1]
                i_0[0,0]=data_1[i+7+j*4,0]
                i_1[0,0]=data[i+7+j*4,1]
                #w_iori=(data_1[i+4+j*4,0]*256+data[i+4+j*4,1])/scale_4[0,0]
                #x_iori=(data_1[i+5+j*4,0]*256+data[i+5+j*4,1])/scale_4[0,0]
                #y_iori=(data_1[i+6+j*4,0]*256+data[i+6+j*4,1])/scale_4[0,0]
                #z_iori=(data_1[i+7+j*4,0]*256+data[i+7+j*4,1])/scale_4[0,0]
                w_iori=(f_0[0,0]*256+f_1[0,0])/scale_4[0,0]
                x_iori=(g_0[0,0]*256+g_1[0,0])/scale_4[0,0]
                y_iori=(h_0[0,0]*256+h_1[0,0])/scale_4[0,0]
                z_iori=(i_0[0,0]*256+i_1[0,0])/scale_4[0,0]
                iori_data[iori_count,1]=w_iori
                iori_data[iori_count,2]=x_iori
                iori_data[iori_count,4]=y_iori # coriと同様に、y,z軸入替
                iori_data[iori_count,3]=z_iori
                iori_count+=1

            if flag_4==0:
                prev_repeat_4=repeat_4
                prev_timestamp_4[0,0]=timestamp_4[0,0]

            if flag_4!=0:
                diff_timestamp_4=(timestamp_4[0,0]-prev_timestamp_4[0,0])/prev_repeat_4
                for j in range(prev_repeat_4):
                    iori_data[iori_count_1,0]=prev_timestamp_4[0,0]+diff_timestamp_4*j
                    iori_count_1+=1

            prev_repeat_4=repeat_4
            prev_timestamp_4[0,0]=timestamp_4[0,0]
            flag_4=1

    return accl_data,gyro_data,cori_data,grav_data,iori_data,unix_data

# realmode
#def bin_extract_5(file_name):# gyro_data,accl_data　x,y順番もとに戻す
def bin_extract_5(path_file):
    #import numpy as np
    # バイナリファイルのパス(binary file path)
    #file_bin = file_name+'.bin'
    file_bin = path_file+'.bin'# "/content/drive/MyDrive/GX010785"+".bin"
    # バイナリファイルを読み込む(read binary file)
    data = np.fromfile(file_bin, dtype=np.uint8) # unsign int8
    data_1 = np.fromfile(file_bin, dtype=np.int8) # sign int8
    # NumPy配列を2次元の形状に変換する(transform into 2 dimension shape)
    data = data.reshape((-1, 2))
    data_1=data_1.reshape((-1,2))

    fourcc_times=0
    ####
    a_0 = np.zeros(1, dtype=np.int64)
    a_1 = np.zeros(1, dtype=np.int64)
    b_0 = np.zeros(1, dtype=np.int64)
    b_1 = np.zeros(1, dtype=np.int64)
    c_0 = np.zeros(1, dtype=np.int64)
    c_1 = np.zeros(1, dtype=np.int64)
    d_0 = np.zeros(1, dtype=np.int64)
    d_1 = np.zeros(1, dtype=np.int64)
    e_0 = np.zeros(1, dtype=np.int64)
    e_1 = np.zeros(1, dtype=np.int64)
    f_0 = np.zeros(1, dtype=np.int64)
    f_1 = np.zeros(1, dtype=np.int64)
    g_0 = np.zeros(1, dtype=np.int64)
    g_1 = np.zeros(1, dtype=np.int64)
    h_0 = np.zeros(1, dtype=np.int64)
    h_1 = np.zeros(1, dtype=np.int64)
    i_0 = np.zeros(1, dtype=np.int64)
    i_1 = np.zeros(1, dtype=np.int64)

    timestamp = np.zeros(1, dtype=np.uint64)
    timestamp_1 = np.zeros(1, dtype=np.uint64)
    timestamp_2 = np.zeros(1, dtype=np.uint64)
    timestamp_3 = np.zeros(1, dtype=np.uint64)
    timestamp_4 = np.zeros(1, dtype=np.uint64)
    prev_timestamp = np.zeros(1, dtype=np.uint64)
    prev_timestamp_1 = np.zeros(1, dtype=np.uint64)
    prev_timestamp_2 = np.zeros(1, dtype=np.uint64)
    prev_timestamp_3 = np.zeros(1, dtype=np.uint64)
    prev_timestamp_4 = np.zeros(1, dtype=np.uint64)

    scale = np.zeros(1, dtype=np.int64)
    scale_1 = np.zeros(1, dtype=np.int64)
    scale_2 = np.zeros(1, dtype=np.int64)
    scale_3 = np.zeros(1, dtype=np.int64)
    scale_4 = np.zeros(1, dtype=np.int64)

    for i in range(data.shape[0]-1):
        FourCC=chr(data[i,0])+chr(data[i,1])
        FourCC+=chr(data[i+1,0])+chr(data[i+1,1])
        if FourCC=='ACCL':
            fourcc_times+=1

    gyro_data=np.zeros([fourcc_times*200,4])
    accl_data=np.zeros([fourcc_times*200,4])
    cori_data=np.zeros([fourcc_times*31,5])
    grav_data=np.zeros([fourcc_times*31,4])
    iori_data=np.zeros([fourcc_times*31,5])
    unix_data=np.zeros([fourcc_times])

    flag=0
    flag_1=0
    gyro_count=0
    gyro_count_1=0
    accl_count=0
    accl_count_1=0
    flag_2=0
    cori_count=0
    cori_count_1=0
    flag_3=0
    grav_count=0
    grav_count_1=0
    flag_4=0
    iori_count=0
    iori_count_1=0
    unix_count=0
    diff_timestamp=0
    diff_timestamp_1=0
    diff_timestamp_2=0
    diff_timestamp_3=0
    diff_timestamp_4=0
    gpsu_times=0
    accl_times=0
    gyro_times=0
    cori_times=0
    grav_times=0
    iori_times=0

    for i in range(data.shape[0]-1):

        FourCC=chr(data[i,0])+chr(data[i,1])
        FourCC+=chr(data[i+1,0])+chr(data[i+1,1])

        if FourCC=='GPSU':
            year=int('20'+chr(data[i+4,0])+chr(data[i+4,1]))
            month=int(chr(data[i+5,0])+chr(data[i+5,1]))
            day=int(chr(data[i+6,0])+chr(data[i+6,1]))
            hour=int(chr(data[i+7,0])+chr(data[i+7,1]))
            minute=int(chr(data[i+8,0])+chr(data[i+8,1]))
            sec=int(chr(data[i+9,0])+chr(data[i+9,1]))
            microsec=int(chr(data[i+10,1])+chr(data[i+11,0])+chr(data[i+11,0]))*1000
            unix_time=datetime.datetime.timestamp(datetime.datetime(year, month, day, hour, minute, sec, microsec))
            unix_data[unix_count] = unix_time+9*60*60
            #print(unix_time)
            unix_count+=1

        if FourCC=='ACCL':
            accl_times += 1
            #timestamp=data[i-46,0]*256**7+data[i-46,1]*256**6+data[i-45,0]*256**5+data[i-45,1]*256**4
            #timestamp+=data[i-44,0]*256**3+data[i-44,1]*256**2+data[i-43,0]*256+data[i-43,1]
            a_0[0]=data[i-46,0]
            a_1[0]=data[i-46,1]
            b_0[0]=data[i-45,0]
            b_1[0]=data[i-45,1]
            c_0[0]=data[i-44,0]
            c_1[0]=data[i-44,1]
            d_0[0]=data[i-43,0]
            d_1[0]=data[i-43,1]
            timestamp[0]=a_0[0]*256**7+a_1[0]*256**6+b_0[0]*256**5+b_1[0]*256**4+c_0[0]*256**3+c_1[0]*256**2+d_0[0]*256+d_1[0]
            repeat=data[i+3,1]
            #scale=data[i-8,0]*256+data[i-8,1]
            e_0[0]=data[i-8,0]
            e_1[0]=data[i-8,1]
            scale[0]=e_0[0]*256+e_1[0]

            for j in range(repeat):
                f_0[0]=data_1[i+4+j*3,0]
                f_1[0]=data[i+4+j*3,1]
                g_0[0]=data_1[i+5+j*3,0]
                g_1[0]=data[i+5+j*3,1]
                h_0[0]=data_1[i+6+j*3,0]
                h_1[0]=data[i+6+j*3,1]
                #x_accl=(data_1[i+4+j*3,0]*256+data[i+4+j*3,1])/scale
                #y_accl=(data_1[i+5+j*3,0]*256+data[i+5+j*3,1])/scale
                #z_accl=(data_1[i+6+j*3,0]*256+data[i+6+j*3,1])/scale
                x_accl=(f_0[0]*256+f_1[0])/scale[0]
                y_accl=(g_0[0]*256+g_1[0])/scale[0]
                z_accl=(h_0[0]*256+h_1[0])/scale[0]

                accl_data[accl_count,3]=x_accl#軸入替、定義に準ずる
                accl_data[accl_count,1]=y_accl
                accl_data[accl_count,2]=z_accl
                accl_count+=1

            if flag==0:
                prev_repeat=repeat
                prev_timestamp[0]=timestamp[0]

            if flag!=0:
                diff_timestamp=(timestamp[0]-prev_timestamp[0])/prev_repeat
                for j in range(prev_repeat):
                    accl_data[accl_count_1,0]=prev_timestamp[0]+diff_timestamp*j
                    accl_count_1+=1

            prev_repeat=repeat
            prev_timestamp[0]=timestamp[0]
            flag=1

            if accl_times == fourcc_times:
                if repeat<197:
                    print("ACCL No.<197 ",repeat)
                    for j in range(repeat):
                        accl_data[accl_count_1,0]=timestamp[0]+diff_timestamp * j
                        accl_count_1+=1
                else:
                    for j in range(prev_repeat):
                        accl_data[accl_count_1, 0] = timestamp[0] + diff_timestamp * j
                        accl_count_1+=1

        if FourCC=='GYRO':
            gyro_times += 1
            #timestamp_1=data[i-46,0]*256**7+data[i-46,1]*256**6+data[i-45,0]*256**5+data[i-45,1]*256**4
            #timestamp_1+=data[i-44,0]*256**3+data[i-44,1]*256**2+data[i-43,0]*256+data[i-43,1]
            a_0[0]=data[i-46,0]
            a_1[0]=data[i-46,1]
            b_0[0]=data[i-45,0]
            b_1[0]=data[i-45,1]
            c_0[0]=data[i-44,0]
            c_1[0]=data[i-44,1]
            d_0[0]=data[i-43,0]
            d_1[0]=data[i-43,1]
            timestamp_1[0]=a_0[0]*256**7+a_1[0]*256**6+b_0[0]*256**5+b_1[0]*256**4+c_0[0]*256**3+c_1[0]*256**2+d_0[0]*256+d_1[0]
            repeat_1=data[i+3,1]
            #scale_1=data[i-8,0]*256+data[i-8,1]
            e_0[0]=data[i-8,0]
            e_1[0]=data[i-8,1]
            scale_1[0]=e_0[0]*256+e_1[0]

            for j in range(repeat_1):
                f_0[0] = data_1[i + 4 + j * 3, 0]
                f_1[0] = data[i + 4 + j * 3, 1]
                g_0[0] = data_1[i + 5 + j * 3, 0]
                g_1[0] = data[i + 5 + j * 3, 1]
                h_0[0] = data_1[i + 6 + j * 3, 0]
                h_1[0] = data[i + 6 + j * 3, 1]
                #x_gyro=(data_1[i+4+j*3,0]*256+data[i+4+j*3,1])/scale_1
                #y_gyro=(data_1[i+5+j*3,0]*256+data[i+5+j*3,1])/scale_1
                #z_gyro=(data_1[i+6+j*3,0]*256+data[i+6+j*3,1])/scale_1
                x_gyro = (f_0[0] * 256 + f_1[0]) / scale_1[0]
                y_gyro = (g_0[0] * 256 + g_1[0]) / scale_1[0]
                z_gyro = (h_0[0] * 256 + h_1[0]) / scale_1[0]

                gyro_data[gyro_count,3]=x_gyro#軸入替、定義に準ずる
                gyro_data[gyro_count,1]=y_gyro
                gyro_data[gyro_count,2]=z_gyro
                gyro_count+=1

            if flag_1==0:
                prev_repeat_1=repeat_1
                prev_timestamp_1[0]=timestamp_1[0]

            if flag_1!=0:
                diff_timestamp_1=(timestamp_1[0]-prev_timestamp_1[0])/prev_repeat_1
                for j in range(prev_repeat_1):
                    gyro_data[gyro_count_1,0]=prev_timestamp_1[0]+diff_timestamp_1*j
                    gyro_count_1+=1

            prev_repeat_1=repeat_1
            prev_timestamp_1[0]=timestamp_1[0]
            flag_1=1

            if gyro_times == fourcc_times:
                if repeat_1<197:
                    print("GYRO No.<197 ", repeat_1)
                    for j in range(repeat_1):
                        gyro_data[gyro_count_1, 0] = timestamp_1[0] + diff_timestamp_1 * j
                        gyro_count_1+=1
                else:
                    for j in range(prev_repeat_1):
                        gyro_data[gyro_count_1, 0] = timestamp_1[0] + diff_timestamp_1 * j
                        gyro_count_1+=1

        if FourCC=='CORI':
            cori_times += 1
            #timestamp_2=data[i-30,0]*256**7+data[i-30,1]*256**6+data[i-29,0]*256**5+data[i-29,1]*256**4
            #timestamp_2+=data[i-28,0]*256**3+data[i-28,1]*256**2+data[i-27,0]*256+data[i-27,1]
            a_0[0]=data[i-30,0]
            a_1[0]=data[i-30,1]
            b_0[0]=data[i-29,0]
            b_1[0]=data[i-29,1]
            c_0[0]=data[i-28,0]
            c_1[0]=data[i-28,1]
            d_0[0]=data[i-27,0]
            d_1[0]=data[i-27,1]
            timestamp_2[0]=a_0[0]*256**7+a_1[0]*256**6+b_0[0]*256**5+b_1[0]*256**4+c_0[0]*256**3+c_1[0]*256**2+d_0[0]*256+d_1[0]
            repeat_2=data[i+3,1]
            #scale_2=data[i-2,0]*256+data[i-2,1]
            e_0[0]=data[i-2,0]
            e_1[0]=data[i-2,1]
            scale_2[0]=e_0[0]*256+e_1[0]

            for j in range(repeat_2):
                f_0[0]=data_1[i+4+j*4,0]
                f_1[0]=data[i+4+j*4,1]
                g_0[0]=data_1[i+5+j*4,0]
                g_1[0]=data[i+5+j*4,1]
                h_0[0]=data_1[i+6+j*4,0]
                h_1[0]=data[i+6+j*4,1]
                i_0[0]=data_1[i+7+j*4,0]
                i_1[0]=data[i+7+j*4,1]
                #w_cori=(data_1[i+4+j*4,0]*256+data[i+4+j*4,1])/scale_2
                #x_cori=(data_1[i+5+j*4,0]*256+data[i+5+j*4,1])/scale_2
                #y_cori=(data_1[i+6+j*4,0]*256+data[i+6+j*4,1])/scale_2
                #z_cori=(data_1[i+7+j*4,0]*256+data[i+7+j*4,1])/scale_2
                w_cori=(f_0[0]*256+f_1[0])/scale_2[0]
                x_cori=(g_0[0]*256+g_1[0])/scale_2[0]
                y_cori=(h_0[0]*256+h_1[0])/scale_2[0]
                z_cori=(i_0[0]*256+i_1[0])/scale_2[0]

                cori_data[cori_count,1]=w_cori
                cori_data[cori_count,2]=x_cori
                cori_data[cori_count,4]=y_cori #GoPro firmwareのバグ？？　y,z軸入替
                cori_data[cori_count,3]=z_cori
                cori_count+=1

            if flag_2==0:
                prev_repeat_2=repeat_2
                prev_timestamp_2[0]=timestamp_2[0]

            if flag_2!=0:
                diff_timestamp_2=(timestamp_2[0]-prev_timestamp_2[0])/prev_repeat_2
                for j in range(prev_repeat_2):
                    cori_data[cori_count_1,0]=prev_timestamp_2[0]+diff_timestamp_2*j
                    cori_count_1+=1

            prev_repeat_2=repeat_2
            prev_timestamp_2[0]=timestamp_2[0]
            flag_2=1

            if cori_times == fourcc_times:  
                if repeat_2<30:
                    print("CORI No.<30 ",repeat_2)
                    for j in range(repeat_2):
                        cori_data[cori_count_1,0] = timestamp_2[0] + diff_timestamp_2 * j
                        cori_count_1+=1
                else:
                    for j in range(prev_repeat_2):
                        cori_data[cori_count_1, 0] = timestamp_2[0] + diff_timestamp_2 * j
                        cori_count_1+=1

        if FourCC=='GRAV':
            grav_times += 1
            #timestamp_3=data[i-28,0]*256**7+data[i-28,1]*256**6+data[i-27,0]*256**5+data[i-27,1]*256**4
            #timestamp_3+=data[i-26,0]*256**3+data[i-26,1]*256**2+data[i-25,0]*256+data[i-25,1]
            a_0[0]=data[i-28,0]
            a_1[0]=data[i-28,1]
            b_0[0]=data[i-27,0]
            b_1[0]=data[i-27,1]
            c_0[0]=data[i-26,0]
            c_1[0]=data[i-26,1]
            d_0[0]=data[i-25,0]
            d_1[0]=data[i-25,1]
            timestamp_3[0]=a_0[0]*256**7+a_1[0]*256**6+b_0[0]*256**5+b_1[0]*256**4+c_0[0]*256**3+c_1[0]*256**2+d_0[0]*256+d_1[0]
            repeat_3=data[i+3,1]
            #scale_3=data[i-2,0]*256+data[i-2,1]
            e_0[0]=data[i-2,0]
            e_1[0]=data[i-2,1]
            scale_3[0]=e_0[0]*256+e_1[0]

            for j in range(repeat_3):
                f_0[0]=data_1[i+4+j*3,0]
                f_1[0]=data[i+4+j*3,1]
                g_0[0]=data_1[i+5+j*3,0]
                g_1[0]=data[i+5+j*3,1]
                h_0[0]=data_1[i+6+j*3,0]
                h_1[0]=data[i+6+j*3,1]
                #x_grav=(data_1[i+4+j*3,0]*256+data[i+4+j*3,1])/scale_3
                #y_grav=(data_1[i+5+j*3,0]*256+data[i+5+j*3,1])/scale_3
                #z_grav=(data_1[i+6+j*3,0]*256+data[i+6+j*3,1])/scale_3
                x_grav=(f_0[0]*256+f_1[0])/scale_3[0]
                y_grav=(g_0[0]*256+g_1[0])/scale_3[0]
                z_grav=(h_0[0]*256+h_1[0])/scale_3[0]

                grav_data[grav_count,1]=x_grav
                grav_data[grav_count,2]=y_grav
                grav_data[grav_count,3]=z_grav

                grav_count+=1

            if flag_3==0:
                prev_repeat_3=repeat_3
                prev_timestamp_3[0]=timestamp_3[0]

            if flag_3!=0:
                diff_timestamp_3=(timestamp_3[0]-prev_timestamp_3[0])/prev_repeat_3
                for j in range(prev_repeat_3):
                    grav_data[grav_count_1,0]=prev_timestamp_3[0]+diff_timestamp_3*j
                    grav_count_1+=1

            prev_repeat_3=repeat_3
            prev_timestamp_3[0]=timestamp_3[0]
            flag_3=1

            if grav_times == fourcc_times:  
                if repeat_3<30:
                    print("GRAV No.<30 ", repeat_3)
                    for j in range(repeat_3):
                        grav_data[grav_count_1,0] = timestamp_3[0] + diff_timestamp_3 * j
                        grav_count_1+=1
                else:
                    for j in range(prev_repeat_3):
                        grav_data[grav_count_1, 0] = timestamp_3[0] + diff_timestamp_3 * j
                        grav_count_1+=1

        if FourCC=='IORI':
            iori_times += 1
            #timestamp_4=data[i-28,0]*256**7+data[i-28,1]*256**6+data[i-27,0]*256**5+data[i-27,1]*256**4
            #timestamp_4+=data[i-26,0]*256**3+data[i-26,1]*256**2+data[i-26,0]*256+data[i-26,1]
            a_0[0]=data[i-28,0]
            a_1[0]=data[i-28,1]
            b_0[0]=data[i-27,0]
            b_1[0]=data[i-27,1]
            c_0[0]=data[i-26,0]
            c_1[0]=data[i-26,1]
            d_0[0]=data[i-25,0]
            d_1[0]=data[i-25,1]
            timestamp_4[0]=a_0[0]*256**7+a_1[0]*256**6+b_0[0]*256**5+b_1[0]*256**4+c_0[0]*256**3+c_1[0]*256**2+d_0[0]*256+d_1[0]
            repeat_4=data[i+3,1]
            # scale_4=data[i-2,0]*256+data[i-2,1]
            e_0[0] = data[i - 2, 0]
            e_1[0] = data[i - 2, 1]
            scale_4[0] = e_0[0] * 256 + e_1[0]

            for j in range(repeat_4):
                f_0[0]=data_1[i+4+j*4,0]
                f_1[0]=data[i+4+j*4,1]
                g_0[0]=data_1[i+5+j*4,0]
                g_1[0]=data[i+5+j*4,1]
                h_0[0]=data_1[i+6+j*4,0]
                h_1[0]=data[i+6+j*4,1]
                i_0[0]=data_1[i+7+j*4,0]
                i_1[0]=data[i+7+j*4,1]
                #w_iori=(data_1[i+4+j*4,0]*256+data[i+4+j*4,1])/scale_4
                #x_iori=(data_1[i+5+j*4,0]*256+data[i+5+j*4,1])/scale_4
                #y_iori=(data_1[i+6+j*4,0]*256+data[i+6+j*4,1])/scale_4
                #z_iori=(data_1[i+7+j*4,0]*256+data[i+7+j*4,1])/scale_4
                w_iori=(f_0[0]*256+f_1[0])/scale_4[0]
                x_iori=(g_0[0]*256+g_1[0])/scale_4[0]
                y_iori=(h_0[0]*256+h_1[0])/scale_4[0]
                z_iori=(i_0[0]*256+i_1[0])/scale_4[0]

                iori_data[iori_count,1]=w_iori
                iori_data[iori_count,2]=x_iori
                iori_data[iori_count,4]=y_iori # coriと同様に、y,z軸入替
                iori_data[iori_count,3]=z_iori
                iori_count+=1

            if flag_4==0:
                prev_repeat_4=repeat_4
                prev_timestamp_4[0]=timestamp_4[0]

            if flag_4!=0:
                diff_timestamp_4=(timestamp_4[0]-prev_timestamp_4[0])/prev_repeat_4
                for j in range(prev_repeat_4):
                    iori_data[iori_count_1,0]=prev_timestamp_4[0]+diff_timestamp_4*j
                    iori_count_1+=1

            prev_repeat_4=repeat_4
            prev_timestamp_4[0]=timestamp_4[0]
            flag_4=1

            if iori_times == fourcc_times:  
                if repeat_4<30:
                    print("IORI No.<30 ",repeat_4)
                    for j in range(repeat_4):
                        iori_data[iori_count_1, 0] = timestamp_4[0] + diff_timestamp_4 * j
                        iori_count_1+=1
                else:
                    for j in range(prev_repeat_4):
                        iori_data[iori_count_1, 0] = timestamp_4[0] + diff_timestamp_4 * j
                        iori_count_1+=1

    end_timestamp=0
    end_timestamp_flag=0
    for i in range(accl_data.shape[0]-1):
        if accl_data[i,0]==0 and end_timestamp_flag==0:
            end_timestamp=i
            end_timestamp_flag=1
    print("ACCL No.: ",end_timestamp)

    end_timestamp_1=0
    end_timestamp_flag_1=0
    for i in range(gyro_data.shape[0]-1):
        if gyro_data[i,0]==0 and end_timestamp_flag_1==0:
            end_timestamp_1=i
            end_timestamp_flag_1=1
    print("GYRO No.: ",end_timestamp_1)

    end_timestamp_2=0
    end_timestamp_flag_2=0
    for i in range(cori_data.shape[0]-1):
        if cori_data[i,0]==0 and end_timestamp_flag_2==0:
            end_timestamp_2=i
            end_timestamp_flag_2=1
    print("CORI No.: ",end_timestamp_2)

    end_timestamp_3=0
    end_timestamp_flag_3=0
    for i in range(grav_data.shape[0]-1):
        if grav_data[i,0]==0 and end_timestamp_flag_3==0:
            end_timestamp_3=i
            end_timestamp_flag_3=1
    print("GRAV No.: ",end_timestamp_3)

    end_timestamp_4=0
    end_timestamp_flag_4=0
    for i in range(iori_data.shape[0]-1):
        if iori_data[i,0]==0 and end_timestamp_flag_4==0:
            end_timestamp_4=i
            end_timestamp_flag_4=1
    print("IORI No.: ",end_timestamp_4)
    #print("unix_count",unix_count)

    return accl_data,gyro_data,cori_data,grav_data,iori_data,unix_data

def gc_lpf_half_ele_var(quaternion_array, sigma_frame_no, kernel_range):

    if kernel_range < 0.5:
        kernel_range = 0.5
    if kernel_range >= 1:
        kernel_range = 0.999999
    kernel_no = int(round(3 * sigma_frame_no)) * 2 + 1
    kernel_no_var = int(kernel_no * kernel_range) + 1
    kernel_no_half = int(kernel_no / 2) + 1
    kernel_array = np.arange(kernel_no)
    kernel_array_no = 0
    for i in range(kernel_no):
        kernel_v = 1.0 / np.sqrt(2 * np.pi) / sigma_frame_no * np.exp(
            (i - round(3 * sigma_frame_no)) ** 2 / (- 2 * sigma_frame_no ** 2))
        if i == 0:
            kernel_inv = 1.0 / kernel_v
        kernel_v = int(np.round(kernel_v * kernel_inv))
        kernel_array[i] = kernel_v
        kernel_array_no += kernel_v
    kernel_array_no_var = 0
    for i in range(kernel_no_var):
        kernel_array_no_var += kernel_array[i]
    zeros_array = np.zeros((len(quaternion_array), 4))
    quat_array = np.zeros((len(quaternion_array), 4))
    quaternion_gc = quaternion.as_quat_array(zeros_array)
    for i in range(len(quaternion_array)):
        if i >= kernel_no_half and i < len(quaternion_array) - kernel_no_half:
            quat_convolve = np.quaternion(0, 0, 0, 0)
            for j in range(kernel_no_var):
                quat_convolve.x += quaternion_array[i - kernel_no_half + j].x * kernel_array[j]
                quat_convolve.y += quaternion_array[i - kernel_no_half + j].y * kernel_array[j]
                quat_convolve.z += quaternion_array[i - kernel_no_half + j].z * kernel_array[j]
                quat_convolve.w += quaternion_array[i - kernel_no_half + j].w * kernel_array[j]
            quat_mean = np.quaternion(quat_convolve.w, quat_convolve.x, quat_convolve.y,
                                      quat_convolve.z) / kernel_array_no_var
            quat_mean = np.normalized(quat_mean)
            quat_array[i] = np.array([quat_mean.w, quat_mean.x, quat_mean.y, quat_mean.z])
    for i in range(len(quaternion_array)):
        if i >= kernel_no_half and i < len(quaternion_array) - kernel_no_half:
            quaternion_gc[i] = np.quaternion(quat_array[i][0], quat_array[i][1], quat_array[i][2], quat_array[i][3])
    for i in range(len(quaternion_array)):
        if i < kernel_no_half:
            quaternion_gc[i] = quaternion_gc[kernel_no_half]
        if i >= len(quaternion_array) - kernel_no_half:  # + kernel_no_diff:
            quaternion_gc[i] = quaternion_gc[len(quaternion_array) - kernel_no_half - 1]
    return quaternion_gc

def gc_lpf(quaternion_array, sigma_frame_no):

    kernel_no=int(round(3*sigma_frame_no))*2+1
    kernel_array=np.arange(kernel_no)
    kernel_array_no=0
    for i in range(kernel_no):
        kernel_v =  1.0/np.sqrt(2*np.pi)/sigma_frame_no * np.exp((i - round(3*sigma_frame_no))**2/(- 2*sigma_frame_no**2))
        if i==0:
            kernel_inv=1.0/kernel_v
        kernel_v=int(np.round(kernel_v*kernel_inv))
        kernel_array[i]=kernel_v
        kernel_array_no+=kernel_v
    quaternion_original=list(range(len(quaternion_array)))
    quaternion_gc=list(range(len(quaternion_array)))
    ones_array=np.ones((kernel_array_no,4))
    quaternion_convolve = quaternion.as_quat_array(ones_array)
    for i in range(len(quaternion_array)):
        quaternion_gc[i]=quaternion_array[i]
        quaternion_original[i]=quaternion_array[i]
    for i in range(len(quaternion_array)):
        if i<kernel_no//2:
            quaternion_gc[i]=np.normalized(quaternion_original[i])
        elif i>len(quaternion_array)-kernel_no//2:
            quaternion_gc[i]=np.normalized(quaternion_original[i])
        else:
            a=0
            for j in range(kernel_no):
                for k in range(kernel_array[j]):
                    a+=1
                    quaternion_convolve[a-1]=quaternion_original[i+j-kernel_no//2-1]
            quaternion_gc[i]=quaternion.mean_rotor_in_chordal_metric(quaternion_convolve)
            quaternion_gc[i]=np.normalized(quaternion_gc[i])
    for i in range(len(quaternion_array)):
        if i<kernel_no//2:
            quaternion_gc[i]=quaternion_gc[kernel_no//2]
        if i>len(quaternion_array)-kernel_no//2:
            quaternion_gc[i]=quaternion_gc[len(quaternion_array)-kernel_no//2]
    #print("Gauss Convolution completed:01")
    return quaternion_gc

def gc_lpf_element(x,sigma_size): # element base

    kernel = np.zeros(int(round(3*sigma_size))*2+1)
    for i in range(kernel.shape[0]):
        kernel[i] =  1.0/np.sqrt(2*np.pi)/sigma_size * np.exp((i - round(3*sigma_size))**2/(- 2*sigma_size**2))
    kernel = kernel / kernel.sum()

    x_long = np.zeros(x.shape[0])
    for i in range(x.shape[0]):
        if i>= kernel.shape[0]//2 and i<= x.shape[0]-1-kernel.shape[0]//2:
            x_short=0
            for j in range(kernel.shape[0]):
                x_short+=kernel[j]*x[i-kernel.shape[0]//2+j]
            x_long[i]=x_short
    for i in range(x.shape[0]):
        if i< kernel.shape[0]//2:
            x_long[i]=x_long[kernel.shape[0]//2]
        if i>x.shape[0]-1-kernel.shape[0]//2:
            x_long[i]=x_long[x.shape[0]-1-kernel.shape[0]//2]
    return x_long

def kalman_filter_63baxis_final_beta_xyz_1(time_stamp, gyro_vec, grav_vec, init_quat, frame_kalman_no, offset):
    #import numpy as np
    # print("kalman filter 63baxis final beta_xyz is running")
    time_fact=1        #0.1#100#10#1
    time_fact_1=0.8
    min_row = min([len(gyro_vec), len(grav_vec)])
    quaternion = np.zeros((min_row, 7))
    gravity_vec = np.zeros((min_row, 3))
    grav = np.zeros(3)
    grav0 = np.zeros(3)
    grav1 = np.zeros(3)
    gyro = np.zeros(3)

    # time_fact=1.0　#値を大きくすると収束が速いが若干波打つ
    dt = 1 / 200

    P6 = np.diag([time_fact * 0.174 * dt ** 2, time_fact * 0.174 * dt ** 2, time_fact * 0.174 * dt ** 2,
                  time_fact * 0.174 * dt ** 2, time_fact_1 * 0.174 * dt ** 2, time_fact_1 * 0.174 * dt ** 2,
                  time_fact_1 * 0.174 * dt ** 2])  # 7x7 OKOKO
    Q6 = np.eye(7, 7)
    R6 = time_fact * np.diag([dt ** 2, dt ** 2, dt ** 2])  # 3x3
    G6 = np.diag([0, time_fact * 0.174 * dt ** 2, time_fact * 0.174 * dt ** 2, time_fact * 0.174 * dt ** 2,
                  time_fact_1 * 0.174 * dt ** 2, time_fact_1 * 0.174 * dt ** 2, time_fact_1 * 0.174 * dt ** 2])  # 7x7
    count_6axis = 0
    beta_x = offset[0]
    beta_y = offset[1]
    beta_z = offset[2]
    ts_pre = None

    for i in range(min_row - 1):
        ts = time_stamp[i]
        grav = np.array([grav_vec[i][0], grav_vec[i][1], grav_vec[i][2]])
        gyro = np.array([gyro_vec[i][0], gyro_vec[i][1], gyro_vec[i][2]])


        if i == 0:
            grav0[0], grav0[1], grav0[2] = grav[0], grav[1], grav[2]
            q6 = np.array([[init_quat.w], [init_quat.x], [init_quat.y], [init_quat.z], [0], [0], [0]])
            q3 = np.array([[init_quat.w], [init_quat.x], [init_quat.y], [init_quat.z]])
            for j in range(7):
                quaternion[i][j] = q6[j]
            frame_kalman_no[0] == 1
            gravity_vec[i][0] = grav0[0]
            gravity_vec[i][1] = grav0[1]
            gravity_vec[i][2] = grav0[2]

        if ts_pre is not None:
            dt = ts - ts_pre
            if frame_kalman_no[i] == 1:
                R6 = time_fact * np.diag([dt ** 2, dt ** 2, dt ** 2])
                G6 = np.diag([0, time_fact * 0.174 * dt ** 2, time_fact * 0.174 * dt ** 2, time_fact * 0.174 * dt ** 2,
                              time_fact_1 * 0.174 * dt ** 2, time_fact_1 * 0.174 * dt ** 2,
                              time_fact_1 * 0.174 * dt ** 2])  # 7x7
                if frame_kalman_no[i - 1] == 0:
                    q6[0][0], q6[1][0], q6[2][0], q6[3][0], q6[4][0], q6[5][0], q6[6][
                        0] = 1, 0, 0, 0, 0, 0, 0
                    grav0[0], grav0[1], grav0[2] = grav1[0], grav1[1], grav1[2]
                H6 = calc_H6bxyz(q6, grav0)  # 3x7
                K6 = P6 @ H6.T @ np.linalg.pinv(H6 @ P6 @ H6.T + R6)  ###  7x3=7x7@7x3@(3x7@7x7@7x3+3x3)
                ht6 = calc_h6b(q6, grav0)
                gravity_vec[i][0] = ht6[0]
                gravity_vec[i][1] = ht6[1]
                gravity_vec[i][2] = ht6[2]
                y6 = np.array([[grav[0]], [grav[1]], [grav[2]]])
                q6 = q6 + K6 @ (y6 - ht6)  # 7x1=7x1+7x3@(3x1-3x1)
                P6 = P6 - K6 @ H6 @ P6
                q6 = calc_f6bxyz(dt, gyro, q6)
                F6 = calc_F6bxyz(dt, gyro, q6)
                P6 = F6 @ P6 @ F6.T + G6 @ Q6 @ G6.T
                for j in range(7):
                    quaternion[i][j] = q6[j]
                grav1 = ht6
            elif frame_kalman_no[i] == 0:
                if frame_kalman_no[i - 1] == 1:
                    # q3[0][0],q3[1][0],q3[2][0],q3[3][0]=q6[0][0],q6[1][0],q6[2][0],q6[3][0]
                    q3[0][0], q3[1][0], q3[2][0], q3[3][
                        0] = 1, 0, 0, 0
                    grav0[0], grav0[1], grav0[2] = grav1[0], grav1[1], grav1[2]
                q3 = calc_f_beta(dt, gyro, q3, offset[0], offset[1], offset[2])
                for j in range(4):
                    quaternion[i][j] = q3[j][0]
                ht6 = calc_h6b(q3,
                               grav0)
                grav1 = ht6
                gravity_vec[i][0] = ht6[0]
                gravity_vec[i][1] = ht6[1]
                gravity_vec[i][2] = ht6[2]
                quaternion[i][4] = offset[0]
                quaternion[i][5] = offset[1]
                quaternion[i][6] = offset[2]

        ts_pre = ts

    return gravity_vec

def calc_H6bxyz(q, grav):  # 6x7 ---> 3x5 ---> 3x6 ---> 3x7
    #import numpy as np
    q0, q1, q2, q3 = q[0][0], q[1][0], q[2][0], q[3][0]
    a0, a1, a2 = grav[0], grav[1], grav[2]

    H = np.array([

        [2*a0*q0+2*(-a2*q2+a1*q3), 2*a0*q1+2*(a1*q2+a2*q3),  2*(-a2*q0+a1*q1)-2*a0*q2, 2*(a1*q0+a2*q1)-2*a0*q3,  0,0,0],# 0, 0],
        [2*a1*q0+2*(a2*q1-a0*q3), -2*a1*q1+2*(a2*q0+a0*q2),  2*a1*q2+2*(a0*q1+a2*q3),  2*(-a0*q0+a2*q2)-2*a1*q3, 0,0,0],# 0, 0],
        [2*a2*q0+2*(-a1*q1+a0*q2),-2*a2*q1+2*(-a1*q0+a0*q3),-2*a2*q2+2*(a0*q0+a1*q3),  2*(a0*q1+a1*q2)+2*a2*q3,  0,0,0]#, 0, 0],

    ], dtype=float)
    return H

def calc_h6b(q, grav):  # 6x1 ---> 3x1
    #import numpy as np
    q0, q1, q2, q3 = q[0][0], q[1][0], q[2][0], q[3][0]
    a0, a1, a2 = grav[0], grav[1], grav[2]
    #m0, m1, m2 = magn[0], magn[1], magn[2]

    h = np.array([  # 6x1 ---> 3x1

        [2*(-a2*q0*q2+a1*q1*q2+a1*q0*q3+a2*q1*q3)+a0*(q0**2+q1**2-q2**2-q3**2)],
        [2*(a2*q0*q1+a0*q1*q2-a0*q0*q3+a2*q2*q3)+a1*(q0**2-q1**2+q2**2-q3**2)],
        [2*(-a1*q0*q1+a0*q0*q2+a0*q1*q3+a1*q2*q3)+a2*(q0**2-q1**2-q2**2+q3**2)],

    ], dtype=float)
    return h

def calc_f6bxyz(dt, gyro, q):  # 7x1 ---> 5x1 ---> 6x1
    #import numpy as np
    g0, g1, g2 = gyro[0], gyro[1], gyro[2]
    q0, q1, q2, q3, q4, q5, q6 = q[0][0], q[1][0], q[2][0], q[3][0], q[4][0], q[5][0], q[6][0]

    q = np.array([
        [q0 - dt * q1 * (-q4 + g0) / 2 - dt * q2 * (-q5 + g1) / 2 - dt * q3 * (-q6 + g2) / 2],
        [q1 + dt * q0 * (-q4 + g0) / 2 - dt * q3 * (-q5 + g1) / 2 + dt * q2 * (-q6 + g2) / 2],
        [q2 + dt * q3 * (-q4 + g0) / 2 + dt * q0 * (-q5 + g1) / 2 - dt * q1 * (-q6 + g2) / 2],
        [q3 - dt * q2 * (-q4 + g0) / 2 + dt * q1 * (-q5 + g1) / 2 + dt * q0 * (-q6 + g2) / 2],
        [q4],
        [q5],
        [q6]
        # [q6-q6*b2]
    ], dtype=float)
    return q

def calc_F6bxyz(dt, gyro, q):  # 7x7--->5x5 ---> 6x6
    #import numpy as np
    g0, g1, g2 = gyro[0], gyro[1], gyro[2]
    q0, q1, q2, q3, q4, q5, q6 = q[0][0], q[1][0], q[2][0], q[3][0], q[4][0], q[5][0], q[6][0]
    #q6=0.0
    F = np.array([
        [1,            -dt*(-q4+g0)/2, -dt*(-q5+g1)/2, -dt*(-q6+g2)/2,  dt*q1/2,  dt*q2/2,  dt*q3/2],
        [dt*(-q4+g0)/2, 1,              dt*(-q6+g2)/2, -dt*(-q5+g1)/2, -dt*q0/2,  dt*q3/2, -dt*q2/2],
        [dt*(-q5+g1)/2,-dt*(-q6+g2)/2,              1,  dt*(-q4+g0)/2, -dt*q3/2, -dt*q0/2,  dt*q1/2],
        [dt*(-q6+g2)/2, dt*(-q5+g1)/2, -dt*(-q4+g0)/2,  1            ,  dt*q2/2, -dt*q1/2, -dt*q0/2],
        [0, 0, 0, 0, 1, 0, 0],
        [0, 0, 0, 0, 0, 1, 0],
        [0, 0, 0, 0, 0, 0, 1]
    ], dtype=float)  # ,dtype=float
    return F

def calc_f_beta(dt, gyro, q,beta_x, beta_y, beta_z):  # 4x1
    #import numpy as np
    g0, g1, g2 = gyro[0]-beta_x, gyro[1]-beta_y, gyro[2]-beta_z
    q0, q1, q2, q3 = q[0][0], q[1][0], q[2][0], q[3][0]

    q = np.array([
        [q0 - dt*q1*g0/2-dt*q2*g1/2-dt*q3*g2/2],
        [dt*q0*g0/2 + q1 + dt*q2*g2/2-dt*q3*g1/2],
        [dt*q0*g1/2-dt*q1*g2/2 + q2 + dt*q3*g0/2],
        [dt*q0*g2/2+dt*q1*g1/2-dt*q2*g0/2 + q3]
    ], dtype=float)
    return q

def horizon(grav_vec, horizon_select):
    #import numpy as np
    # 引数
    # grav_vec:重力ベクトル(規格済み)
    # horizon_select: 0--> x-y平面、1--> y軸roll回り、2--> x軸pitch回り、3(else)--> y軸＆x軸回り合成
    # 戻り値
    # 重力四元数の逆四元数を配列に変換したもの
    zeros_array = np.zeros((len(grav_vec), 4))
    zeros_array_1 = np.zeros((len(grav_vec), 4))
    quat_g = quaternion.as_quat_array(zeros_array)
    quat_g_1 = quaternion.as_quat_array(zeros_array_1)
    quat_array = np.zeros((len(grav_vec), 4))
    if horizon_select == 0:
        for i in range(len(grav_vec)):
            theta_x = math.atan(grav_vec[i][1] / grav_vec[i][2])
            theta_y = -math.atan(grav_vec[i][0] / (math.sqrt(grav_vec[i][1] ** 2 + grav_vec[i][2] ** 2)))
            R_x = np.array(
                [[1., 0., 0.], [0., math.cos(theta_x), math.sin(theta_x)], [0., -math.sin(theta_x), math.cos(theta_x)]])
            R_y = np.array(
                [[math.cos(theta_y), 0, -math.sin(theta_y)], [0., 1., 0.], [math.sin(theta_y), 0, math.cos(theta_y)]])
            quat_g[i] = quaternion.from_rotation_matrix(R_x @ R_y, nonorthogonal=True)
            quat_g[i] = np.conjugate(quat_g[i])
            if quat_g[i].w < 0.0:
                quat_g[i] = np.quaternion(-quat_g[i].w, -quat_g[i].x, -quat_g[i].y, -quat_g[i].z)
            quat_g[i] = np.conjugate(quat_g[i])
    elif horizon_select == 1:
        for i in range(len(grav_vec)):

            if abs(grav_vec[i][2]) >= abs(grav_vec[i][0]):
                q_angle = -math.asin(grav_vec[i][0])
                s_0 = math.cos(q_angle / 2)
                s_1 = 0.0
                s_2 = math.sin(q_angle / 2)
                s_3 = 0.0
                quat_g[i] = np.quaternion(s_0, -s_1, -s_2, -s_3)
            else:
                q_angle = math.acos(grav_vec[i][2])
                s_0 = math.cos(q_angle / 2)
                s_1 = 0.0
                if grav_vec[i][0] >= 0:
                    s_2 = math.sin(q_angle / 2)
                else:
                    s_2 = -math.sin(q_angle / 2)
                s_3 = 0.0
                quat_g[i] = np.quaternion(s_0, s_1, s_2, s_3)
    elif horizon_select == 2:
        for i in range(len(grav_vec)):
            q_angle = math.asin(grav_vec[i][1])
            s_0 = math.cos(q_angle / 2)
            s_1 = math.sin(q_angle / 2)
            s_2 = 0.0
            s_3 = 0.0
            quat_g[i] = np.quaternion(s_0, -s_1, -s_2, -s_3)
    elif horizon_select == 3:
        for i in range(len(grav_vec)):
            q_angle = -math.asin(grav_vec[i][0])
            s_0 = math.cos(q_angle / 2)
            s_1 = 0.0
            s_2 = math.sin(q_angle / 2)
            s_3 = 0.0
            quat_g[i] = np.quaternion(s_0, -s_1, -s_2, -s_3)
            q_angle = math.asin(grav_vec_1[i][1])
            t_0 = math.cos(q_angle / 2)
            t_1 = math.sin(q_angle / 2)
            t_2 = 0.0
            t_3 = 0.0
            quat_g_1[i] = np.quaternion(t_0, -t_1, -t_2, -t_3)
            quat_g[i] = quat_g[i] * quat_g_1[i]
    else:
        for i in range(len(grav_vec)):
            quat_g[i] = np.quaternion(1, 0, 0, 0)
    for i in range(len(grav_vec)):
        quat_array[i][0] = quat_g[i].w
        quat_array[i][1] = quat_g[i].x
        quat_array[i][2] = quat_g[i].y
        quat_array[i][3] = quat_g[i].z

    return quat_array

def add_black_border(img, height_border_size, width_border_size):
    #import numpy as np
    height, width = img.shape[0], img.shape[1]
    new_img = np.zeros((height + 2 * height_border_size, width + 2 * width_border_size, 3), dtype=img.dtype)
    new_img[height_border_size:height + height_border_size, width_border_size:width + width_border_size] = img
    return new_img

def stab_horizon(file_name, path_file, horizon_select, quaternion_select, grav_n, timestamp_shift, pitch_offset, dist_parameter):

    import pandas as x_datas
    import numpy as np

    x_datas = x_datas.DataFrame()
    timestamp_shift = float(timestamp_shift)

    cap = cv2.VideoCapture(path_file + '.MP4')
    print('cv2 video capture succeed')
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    
    # 対象動画は,5.3Kのみ
    if width != 5312 or height != 4648:  # 5.3Kではない時
        print('this video is not 5312 x 4648,5.3K')
    else:
        print('this video is 5312 x 4648, 5.3K')
        # sys.exit()
    frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    print('frame count (OpenCV) ',frame_count)
    fps = cap.get(cv2.CAP_PROP_FPS)
    print('fps (OpenCV): ', round(fps,2))

    # 対象は30FPS動画のみ
    if fps < 29.9 or fps > 30.1:
        print('this video is not 30fps')
    else:
        print('this video is 30fps')
        # sys.exit()

    duration_sec = int(frame_count / fps)  # 動画時間（秒）
    print('duration_sec: ', duration_sec)
    #file_bin = file_name + '.bin'
    file_bin = path_file + '.bin'
    data = np.fromfile(file_bin, dtype=np.uint8)  # unsign int8
    data_1 = np.fromfile(file_bin, dtype=np.int8)  # sign int8
    data = data.reshape((-1, 2))
    data_1 = data_1.reshape((-1, 2))

    # GoPro HERO model No.
    print('GoPro HERO model No. is ',chr(data[16,0])+chr(data[16,1]))

    # realspeed or timewarp判定
    accl_size = 0
    for i in range(data.shape[0] - 1):
        FourCC = chr(data[i, 0]) + chr(data[i, 1])
        FourCC += chr(data[i + 1, 0]) + chr(data[i + 1, 1])
        if FourCC == 'ACCL' and accl_size == 0:
            repeat = data[i + 3, 1]
            print('GYRO(ACCL) data number per sec: ', repeat)
            accl_size += 1
    if repeat > 195:
        mode = 'realspeed'
    else:
        mode = 'timewarp'

    cori_data = np.zeros([duration_sec * 31, 5])
    grav_data = np.zeros([duration_sec * 31, 4])
    iori_data = np.zeros([duration_sec * 31, 5])
    unix_data = np.zeros(duration_sec)

    if mode == 'timewarp':
        print('this video is ', mode)
        gyro_data = np.zeros([duration_sec * 31, 4])
        accl_data = np.zeros([duration_sec * 31, 4])
        #accl_data, gyro_data, cori_data, grav_data, iori_data, unix_data = bin_extract_4(file_name)
        accl_data, gyro_data, cori_data, grav_data, iori_data, unix_data = bin_extract_4(path_file)
    else:
        # mode='realspeed'
        print('this video is ', mode)
        gyro_data = np.zeros([duration_sec * 200, 4])
        accl_data = np.zeros([duration_sec * 200, 4])
        #accl_data, gyro_data, cori_data, grav_data, iori_data, unix_data = bin_extract_5(file_name)
        accl_data, gyro_data, cori_data, grav_data, iori_data, unix_data = bin_extract_5(path_file)

    end_timestamp = 0  # ACCL, GYROデータのトータル行数
    end_timestamp_flag = 0
    for i in range(accl_data.shape[0] - 1):
        if accl_data[i, 0] == 0 and end_timestamp_flag == 0:
            end_timestamp = i
            end_timestamp_flag = 1
    print('ACCL,GYRO number(end_timestamp): ', end_timestamp)
    
    end_timestamp_1 = 0  # CORI, GRAV, IORIデータのトータル行数
    end_timestamp_flag_1 = 0
    for i in range(cori_data.shape[0] - 1):
        if cori_data[i, 0] == 0 and end_timestamp_flag_1 == 0:
            end_timestamp_1 = i
            end_timestamp_flag_1 = 1
    print('CORI,GRAV,IORI number(end_timestamp_1): ', end_timestamp_1)

    # metadataの書き出し
    for i in range(end_timestamp):
        gyro_data[i][0] = gyro_data[i][0] / 1000000  # unit sec
        x_datas.loc[i, "timestamp(GYRO)"] = gyro_data[i][0]
        x_datas.loc[i, "GYRO x"] = gyro_data[i][1]
        x_datas.loc[i, "GYRO y"] = gyro_data[i][2]
        x_datas.loc[i, "GYRO z"] = gyro_data[i][3]
        accl_data[i][0] = accl_data[i][0] / 1000000  # unit sec
        x_datas.loc[i, "timedtamp(ACCL)"] = accl_data[i][0]
        x_datas.loc[i, "ACCL x"] = accl_data[i][1]
        x_datas.loc[i, "ACCL y"] = accl_data[i][2]
        x_datas.loc[i, "ACCL z"] = accl_data[i][3]
    for i in range(end_timestamp_1):
        cori_data[i][0] = cori_data[i][0] / 1000000  # unit sec
        x_datas.loc[i, "timestamp(CORI)"] = cori_data[i][0]
        x_datas.loc[i, "CORI q1"] = cori_data[i][2]
        x_datas.loc[i, "CORI q2"] = cori_data[i][3]
        x_datas.loc[i, "CORI q3"] = cori_data[i][4]
        x_datas.loc[i, "CORI q0"] = cori_data[i][1]
        grav_data[i][0] = grav_data[i][0] / 1000000  # unit sec
        x_datas.loc[i, "timestamp(GRAV)"] = grav_data[i][0]
        x_datas.loc[i, "GRAV x"] = grav_data[i][1]
        x_datas.loc[i, "GRAV z"] = grav_data[i][3]
        x_datas.loc[i, "GRAV y"] = grav_data[i][2]
        iori_data[i][0] = iori_data[i][0] / 1000000  # unit sec
        x_datas.loc[i, "timestamp(IORI)"] = iori_data[i][0]
        x_datas.loc[i, "IORI q1"] = iori_data[i][2]
        x_datas.loc[i, "IORI q2"] = iori_data[i][3]
        x_datas.loc[i, "IORI q3"] = iori_data[i][4]
        x_datas.loc[i, "IORI q0"] = iori_data[i][1]

    # 手振れ補正有無の判定（手振れ補正なしでは、IORI四元数データは、(1,0,0,0))
    if iori_data[0][2] > 0.01 or iori_data[0][3] > 0.01 or iori_data[0][4] > 0.01:
        print('this video has been alraedy electrically stabilized')
        # sys.exit()

    # pitch & roll オフセット
    roll_offset = 0  # 単位：度
    pitch_offset = math.sin(pitch_offset / 180 * math.pi / 2)  # deg to rad
    roll_offset = math.sin(roll_offset / 180 * math.pi / 2)  # deg to rad
    pitch_offset_cos = math.cos(pitch_offset / 180 * math.pi / 2)  # deg to rad
    roll_offset_cos = math.cos(roll_offset / 180 * math.pi / 2)  # deg to rad
    quat_pitch = np.quaternion(pitch_offset_cos, pitch_offset, 0.0, 0.0)
    quat_roll = np.quaternion(roll_offset_cos, 0.0, 0.0, roll_offset)  # y,z入替

    if mode == 'realspeed':
        time_stamp = np.zeros(end_timestamp)
        zeros_array_0 = np.zeros((end_timestamp, 4))
        zeros_array_1 = np.zeros((end_timestamp, 4))
        quat_q = quaternion.as_quat_array(zeros_array_0)  # 四元数
        quat_w = quaternion.as_quat_array(zeros_array_1)

        # 四元数の計算（200Hz角速度の時間積分）
        for i in range(end_timestamp):
            quat_w[i] = np.quaternion(0.0, gyro_data[i][1], gyro_data[i][2], gyro_data[i][3])  # 角速度ベクトルの四元数表現
            time_stamp[i] = gyro_data[i][0]  # unit sec
        quat_q[0] = np.quaternion(1.0, 0.0, 0.0, 0.0)  # 初期値
        for i in range(end_timestamp - 1):
            quat_q[i + 1] = quat_q[i] + (time_stamp[i + 1] - time_stamp[i]) * quat_q[i] * quat_w[
                i] / 2.0  # 角速度/四元数微分方程式の離散式表現
        for i in range(end_timestamp - 1):
            quat_q[i] = np.normalized(quat_q[i])  # 規格化
        # 四元数（200Hz)の平滑化
        sigma_frame_no_q = 30  # 30Hzデータの場合　15,or 30
        sigma_200 = int(sigma_frame_no_q * 200 / 30)  # 200Hzデータの場合に換算
        kernel_range = 1.0  # 0.5,1.0
        zeros_array_2 = np.zeros((end_timestamp, 4))
        quat_q_smo = quaternion.as_quat_array(zeros_array_2)
        quat_q_smo = gc_lpf_half_ele_var(quat_q, sigma_200, kernel_range) # 四元数の平滑化処理
        # ACCL（加速度センサ）の四元数化 --> ガウス畳み込みによる平滑処理
        zeros_array_3 = np.zeros((end_timestamp, 4))
        zeros_array_4 = np.zeros((end_timestamp, 4))
        quat_a = quaternion.as_quat_array(zeros_array_3)
        quat_a_gc = quaternion.as_quat_array(zeros_array_4)
        for i in range(end_timestamp):
            quat_a[i] = np.quaternion(0.0, accl_data[i][1], accl_data[i][2], accl_data[i][3])  # 加速度ベクトルの四元数表現
        sigma_frame_no = 5#100
        quat_a_gc = gc_lpf(quat_a, sigma_frame_no)# 加速度四元数の平滑化
        # ACCL(200Hz)の絶対値定義
        gyro_vec = np.zeros((end_timestamp, 3))
        accl_vec = np.zeros((end_timestamp, 3))
        gyro_abs = np.zeros(end_timestamp)
        accl_abs = np.zeros(end_timestamp)
        for i in range(end_timestamp):
            gyro_vec[i] = [gyro_data[i][1], gyro_data[i][2], gyro_data[i][3]]
            accl_vec[i] = [accl_data[i][1], accl_data[i][2], accl_data[i][3]]
            accl_abs[i] = np.linalg.norm(accl_vec[i])  # 加速度ベクトルの絶対値（大きさ）
        sigma_size = 40
        accl_lpf = gc_lpf_element(accl_abs, sigma_size)# 加速度の大きさの平滑化
        for i in range(end_timestamp):
            accl_vec[i] = [quat_a_gc[i].x, quat_a_gc[i].y, quat_a_gc[i].z]  # 平滑化と同時に規格化
            accl_abs[i] = np.linalg.norm(accl_vec[i])  # 平滑後の加速度の大きさ

        # 静止状態の判定
        frame_kalman_no = np.zeros(end_timestamp)
        frame_kalman_no_1 = np.zeros(end_timestamp)
        accl_center = 9.85  # 重力加速度のセンター値
        accl_up_limit = accl_center * 1.02  # 上限値
        accl_lo_limit = accl_center * 0.98  # 下限値
        for i in range(end_timestamp):
            if accl_lpf[i] < accl_up_limit and accl_lpf[i] > accl_lo_limit:
                frame_kalman_no[i] = 1  # 静止状態とみなす　Kalman Filterによる6軸計算
            else:
                frame_kalman_no[i] = 0  # 運動加速度が無視できないとする　角速度のみ3軸計算
            x_datas.loc[i, "Kalman / 3-axis"] = frame_kalman_no[i]

        grav_vec = np.zeros((end_timestamp, 4))
        for i in range(end_timestamp):  # 規格化された加速度ベクトルを重力ベクトルとおく
            grav_vec[i][0] = accl_vec[i][0]
            grav_vec[i][1] = accl_vec[i][1]
            grav_vec[i][2] = accl_vec[i][2]

        # kalman filterによる重力ベクトル(200Hz)の生成
        init_quat = np.quaternion(1.0, 0.0, 0.0, 0.0)  # 初期値
        offset_x = 0.0
        offset_y = 0.0
        offset_z = 0.0
        offset = np.array([offset_x, offset_y, offset_z])  # 角速度のオフセット値
        gravity_vec = kalman_filter_63baxis_final_beta_xyz_1(time_stamp, gyro_vec, grav_vec, init_quat,frame_kalman_no, offset)
        zeros_array_5 = np.zeros((end_timestamp, 4))
        quat_g_vec200 = quaternion.as_quat_array(zeros_array_5)
        zeros_array_6 = np.zeros((end_timestamp, 4))
        quat_g_vec200_xyz = quaternion.as_quat_array(zeros_array_6)
        for i in range(end_timestamp - 1):  ##同じ？
            quat_g_vec200_xyz[i] = np.quaternion(0.0, gravity_vec[i][0], gravity_vec[i][1],
                                                 gravity_vec[i][2])  # 前述の重力ベクトルの四元数表現
            quat_g_vec200_xyz[i] = np.normalized(quat_g_vec200_xyz[i])  # 規格化

        # 四元数のSLERPによる200Hsから30Hsデータへの間引き
        zeros_array_7 = np.zeros((end_timestamp_1, 4))  # 行数は30Hz
        zeros_array_8 = np.zeros((end_timestamp_1, 4))
        quat_q_thin_1 = quaternion.as_quat_array(zeros_array_7)
        quat_cori_diff_1 = quaternion.as_quat_array(zeros_array_8)
        zeros_array_9 = np.zeros((end_timestamp, 4))  # ここだけ行数は200Hz
        quat_accl = quaternion.as_quat_array(zeros_array_9)
        zeros_array_10 = np.zeros((end_timestamp_1, 4))
        quat_g_vec_thin_xyz = quaternion.as_quat_array(zeros_array_10)
        time_stamp_thin_1 = np.zeros(end_timestamp_1)
        for i in range(end_timestamp):
            quat_accl[i] = np.quaternion(0, accl_data[i][1], accl_data[i][2], accl_data[i][3])  # 200Hz加速度ベクトルの四元数表現
        timestamp_shift = timestamp_shift / fps  # δを時間差（秒）に換算
        for i in range(end_timestamp_1):
            if i == 0:
                quat_q_thin_1[0] = quat_q[0]
                quat_g_vec_thin_xyz[0] = np.quaternion(quat_g_vec200_xyz[0].w, quat_g_vec200_xyz[0].x,
                                                       quat_g_vec200_xyz[0].y, quat_g_vec200_xyz[0].z)
            else:
                current_ts = i / fps + gyro_data[0][0] + timestamp_shift
                for j in range(end_timestamp - 1):
                    if gyro_data[j][0] >= current_ts:  # and gyro_data[j+1][0]<current_ts:
                        t_1 = 0
                        t_2 = gyro_data[j][0] - gyro_data[j - 1][0]
                        t_x = current_ts - gyro_data[j - 1][0]
                        quat_q_thin_1[i] = quaternion.slerp(quat_q[j - 1], quat_q[j], t_1, t_2,
                                                            t_x)  # GoProのgyroの積分 needs
                        quat_g_vec_thin_xyz[i] = quaternion.slerp(quat_g_vec200_xyz[j - 1], quat_g_vec200_xyz[j], t_1,
                                                                  t_2, t_x)  # 63xyz kalmanで推定した重力ベクトル
                        time_stamp_thin_1[i] = gyro_data[j - 1][0] + t_x
                        break
        for i in range(end_timestamp_1 - 1):
            x_datas.loc[i, "quaternion q1(by GYRO)"] = quat_q_thin_1[i].x
            x_datas.loc[i, "quaternion q2(by GYRO)"] = quat_q_thin_1[i].y
            x_datas.loc[i, "quaternion q3(by GYRO)"] = quat_q_thin_1[i].z
            x_datas.loc[i, "quaternion q0(by GYRO)"] = quat_q_thin_1[i].w
            x_datas.loc[i, "grav x(by Kalman)"] = quat_g_vec_thin_xyz[i].x
            x_datas.loc[i, "grav y(by Kalman)"] = quat_g_vec_thin_xyz[i].y
            x_datas.loc[i, "grav z(by Kalman)"] = quat_g_vec_thin_xyz[i].z
            
        # GYRO同期調整δの評価
        zeros_array_11 = np.zeros((end_timestamp_1, 4))
        quat_q_cori = quaternion.as_quat_array(zeros_array_11)
        for i in range(end_timestamp_1 - 1):
            quat_q_cori[i] = np.quaternion(cori_data[i][1], cori_data[i][2], cori_data[i][3],
                                           cori_data[i][4])  # CORIデータの四元数表記
            quat_cori_diff_1[i] = quat_q_thin_1[i] * np.conjugate(quat_q_cori[i])  # CORIデータとの「割算」
            x_datas.loc[i, "quat / CORI q1"] = quat_cori_diff_1[i].x
            x_datas.loc[i, "quat / CORI q2"] = quat_cori_diff_1[i].y
            x_datas.loc[i, "quat / CORI q3"] = quat_cori_diff_1[i].z
            x_datas.loc[i, "quat / CORI q0"] = quat_cori_diff_1[i].w

        # grav_vec_0 の生成 metadataの重力ベクトルの軸変換と平滑化
        zeros_array_12 = np.zeros((end_timestamp_1, 4))
        quat_grav_vec = quaternion.as_quat_array(zeros_array_12)
        for i in range(end_timestamp_1):
            quat_grav_vec[i] = np.quaternion(0.0, grav_data[i][1], grav_data[i][3],
                                             grav_data[i][2])  # メタデータGRAVデータの四元数表式　//y,z exchange
        sigma_frame_no_g = 30
        quat_grav_gc = gc_lpf(quat_grav_vec, sigma_frame_no_g) # GRAVの平滑化　 //30#10#strokeでは10がよい####
        grav_vec_0 = np.zeros((end_timestamp_1, 3))
        for i in range(end_timestamp_1):
            grav_vec_0[i] = np.array([quat_grav_gc[i].x, quat_grav_gc[i].y, quat_grav_gc[i].z])  # 平滑化後のGRAVデータ

        # grav_vec_3 の生成 36axis_xyzで求めた重力ベクトルに対してガウス畳み込みによる平滑処理＆規格化
        zeros_array_13 = np.zeros((end_timestamp_1, 4))
        quat_g_gc_xyz = quaternion.as_quat_array(zeros_array_13)
        zeros_array_14 = np.zeros((end_timestamp_1, 4))
        quat_gc_gc_xyz = quaternion.as_quat_array(zeros_array_14)
        for i in range(end_timestamp_1):
            quat_g_gc_xyz[i] = np.quaternion(0.0, quat_g_vec_thin_xyz[i].x, quat_g_vec_thin_xyz[i].y,
                                             quat_g_vec_thin_xyz[i].z)
        quat_gc_gc_xyz = gc_lpf(quat_g_gc_xyz, sigma_frame_no_g)# 規格化されている
        grav_vec_3 = np.zeros((end_timestamp_1, 3))
        for i in range(end_timestamp_1):
            grav_vec_3[i] = np.array([quat_gc_gc_xyz[i].x, quat_gc_gc_xyz[i].y, quat_gc_gc_xyz[i].z])  # ベクトル表記に戻す
        
        # 重力ベクトルの選択 0 or 3
        # grav_n=3
        # grav_vec_0-->original grav vec in meta,  grav_vec_3-->63 axis beta xyz
        zeros_array_15 = np.zeros((end_timestamp_1, 4))
        quat_horizon_inv = quaternion.as_quat_array(zeros_array_15)
        quat_horizon = np.zeros((end_timestamp_1, 4))
        grav_vec_n = np.zeros((end_timestamp_1, 3))
        grav_vec_1_n = np.zeros((end_timestamp_1, 3))

        if grav_n == 0:  # original gravity vec in metadata
            for i in range(end_timestamp_1):
                grav_vec_n[i] = np.array([grav_vec_0[i][0], grav_vec_0[i][1], grav_vec_0[i][2]])  # y,z入替
        elif grav_n == 3:  # 63kalman xyz gravity vec
            for i in range(end_timestamp_1):
                grav_vec_n[i] = np.array([grav_vec_3[i][0], grav_vec_3[i][1], grav_vec_3[i][2]])  # y,z入替
        else:  # same as #0
            for i in range(end_timestamp_1):
                grav_vec_n[i] = np.array([grav_vec_0[i][0], grav_vec_0[i][1], grav_vec_0[i][2]])  # y,z入替

        # horizon_select=0 # 0--> x-y平面、1--> y軸roll回り、2--> x軸pitch回り、3--> y軸＆x軸回り合成, 4(else)--> none
        quat_horizon = horizon(grav_vec_n, horizon_select)# 水平維持四元数（厳密には逆四元数）戻り値は配列
        for i in range(end_timestamp_1 - 1):
            quat_horizon_inv[i] = np.quaternion(quat_horizon[i][0], quat_horizon[i][1], quat_horizon[i][3],
                                                quat_horizon[i][2])  # 配列を四元数表記に変換
        for i in range(end_timestamp_1 - 1):
            quat_q_thin_1[i] = np.quaternion(quat_q_thin_1[i].w, quat_q_thin_1[i].x, quat_q_thin_1[i].z,
                                             quat_q_thin_1[i].y)  # y,z入替
        zeros_array_16 = np.zeros((end_timestamp_1, 4))
        quat_smo_thin = quaternion.as_quat_array(zeros_array_16)
        quat_smo_thin = gc_lpf(quat_q_thin_1, sigma_frame_no_q) # Kalmanで求めた30Hz四元数を平滑化
        # sigma_frame_no_q= 30
        zeros_array_17 = np.zeros((end_timestamp_1, 4))
        quat_q_cori_1 = quaternion.as_quat_array(zeros_array_17)
        for i in range(end_timestamp_1 - 1):
            quat_q_cori_1[i] = np.quaternion(cori_data[i][1], cori_data[i][2], cori_data[i][4],
                                             cori_data[i][3])  # <A>オリジナル y <--> z
            quat_q_cori_1[i] = np.normalized(quat_q_cori_1[i])  # CORIデータの四元数化と規格化
        
        # CORIデータの平滑化    
        zeros_array_18 = np.zeros((end_timestamp_1, 4))
        quat_smo_cori_1 = quaternion.as_quat_array(zeros_array_18)  # <A>
        quat_smo_cori_1 = gc_lpf_half_ele_var(quat_q_cori_1, sigma_frame_no_q, kernel_range)
        for i in range(end_timestamp_1 - 1):
            x_datas.loc[i, "cori_smoothed q1"] = quat_smo_cori_1[i].x
            x_datas.loc[i, "cori_smoothed q2"] = quat_smo_cori_1[i].z
            x_datas.loc[i, "cori_smoothed q3"] = quat_smo_cori_1[i].y
            x_datas.loc[i, "cori_smoothed q0"] = quat_smo_cori_1[i].w
            x_datas.loc[i, "quat_smoothed q1"] = quat_smo_thin[i].x
            x_datas.loc[i, "quat_smoothed q2"] = quat_smo_thin[i].z
            x_datas.loc[i, "quat_smoothed q3"] = quat_smo_thin[i].y
            x_datas.loc[i, "qyat_smoothed q0"] = quat_smo_thin[i].w    
            
        # 映像変換に引き渡す四元数の計算
        zeros_array_19 = np.zeros((end_timestamp_1, 4))
        cori_crop_1 = quaternion.as_quat_array(zeros_array_19)
        # quaternion_select=2 # 1:CORI 2:GYRO
        if quaternion_select == 2:
            for i in range(end_timestamp_1 - 1):
                cori_crop_1[i] = quat_roll * quat_pitch * quat_horizon_inv[i] * quat_smo_thin[i] * np.conjugate(
                    quat_q_thin_1[i])
        else:  # 1
            for i in range(end_timestamp_1 - 1):
                cori_crop_1[i] = quat_roll * quat_pitch * quat_horizon_inv[i] * quat_smo_cori_1[i] * np.conjugate(
                    quat_q_cori_1[i])
        for i in range(end_timestamp_1 - 1):
            x_datas.loc[i, "quat_out q1"] = cori_crop_1[i].x
            x_datas.loc[i, "quat_out q2"] = cori_crop_1[i].z
            x_datas.loc[i, "quat_out q3"] = cori_crop_1[i].y
            x_datas.loc[i, "quat_out q0"] = cori_crop_1[i].w
            
        # 映像の歪度（0側：リニア、1：歪最大）
        dist_para = np.ones(end_timestamp_1)
        for i in range(end_timestamp_1 - 1):
            dist_para[i] = dist_parameter

    ##########################################################################################
    else:  # timewarp　このモードは、水平維持機能のみ
        # horizon_select=0 # 0--> x-y平面、1--> y軸（roll軸）
        grav_n = 0  # grav_vec_0-->original grav vec in meta　オリジナルのGRAVを使用
        zeros_array_20 = np.zeros((end_timestamp_1, 4))
        quat_grav_vec = quaternion.as_quat_array(zeros_array_20)

        for i in range(end_timestamp_1):  # 重力ベクトルの四元数表記
            quat_grav_vec[i] = np.quaternion(0.0, grav_data[i][1], grav_data[i][3], grav_data[i][2])  # y,z exchange

        quat_grav_gc_5 = gc_lpf(quat_grav_vec, 10)  # 平滑化パラメータの違う2種類を生成
        quat_grav_gc_1 = gc_lpf(quat_grav_vec, 30)
        grav_vec_0 = np.zeros((end_timestamp_1, 3))
        for i in range(end_timestamp_1):  ## end_timestamp_1-1にするとNG
            if grav_data[i + 1][0] - grav_data[i][0] > 0.05:  # timewarpx5
                grav_vec_0[i] = np.array([quat_grav_gc_5[i].x, quat_grav_gc_5[i].y, quat_grav_gc_5[i].z])
            else:  # realspeed
                grav_vec_0[i] = np.array([quat_grav_gc_1[i].x, quat_grav_gc_1[i].y, quat_grav_gc_1[i].z])

        grav_vec_n = np.zeros((end_timestamp_1, 3))
        if grav_n == 0:  # original gravity vec in metadata
            for i in range(end_timestamp_1):
                grav_vec_n[i] = np.array([grav_vec_0[i][0], grav_vec_0[i][1], grav_vec_0[i][2]])  # y,z入替

        quat_horizon_cori_xy = np.zeros((end_timestamp_1, 4))
        horizon_select = 0  # 水平維持軸：xy平面 timewarpの時
        quat_horizon_cori_xy = horizon(grav_vec_n, horizon_select)
        quat_horizon_cori_roll = np.zeros((end_timestamp_1, 4))
        horizon_select = 1  # 水平維持軸：ｙ軸（roll軸）realspeedの時
        quat_horizon_cori_roll = horizon(grav_vec_n, horizon_select)
        zeros_array_21 = np.zeros((end_timestamp_1, 4))
        quat_horizon_inv = quaternion.as_quat_array(zeros_array_21)
        dist_para = np.ones(end_timestamp_1)
        # 水平維持の四元数
        for i in range(end_timestamp_1 - 1):
            if grav_data[i + 1][0] - grav_data[i][0] > 0.05:  # timewarpx5の時
                quat_horizon_inv[i] = np.quaternion(quat_horizon_cori_xy[i][0], quat_horizon_cori_xy[i][1],
                                                    quat_horizon_cori_xy[i][3], quat_horizon_cori_xy[i][2])
                dist_para[i] = 1.0  # 歪最大
            else:  # realspeedの時
                quat_horizon_inv[i] = np.quaternion(quat_horizon_cori_roll[i][0], quat_horizon_cori_roll[i][1],
                                                    quat_horizon_cori_roll[i][3], quat_horizon_cori_roll[i][2])
                dist_para[i] = dist_parameter  # リニア映像側

        zeros_array_22 = np.zeros((end_timestamp_1, 4))
        cori_crop_1 = quaternion.as_quat_array(zeros_array_22)
        for i in range(end_timestamp_1 - 1):
            cori_crop_1[i] = quat_roll * quat_pitch * quat_horizon_inv[
                i]  # *quat_smo_cori_1[i]*np.conjugate(quat_q_cori_1[i])
    #x_datas.to_csv(file_name+'.csv')
    x_datas.to_csv(path_file+'.csv')# "/content/drive/MyDrive/my_drive/GX010785"+".csv"
    print('quaternion calc done')
    return cori_crop_1,dist_para

def stabilization_53_87of18(file_name, path_file, quat_crop, dist_para):  # 5.3K8:7 Wide with border

    # GoPro 4K premiere pro 書き込み動画の読み込み
    #cap = cv2.VideoCapture(file_name + '.MP4')
    cap = cv2.VideoCapture(path_file + '.MP4')
    # 動画情報取得
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    # 動画の書き出し設定
    fmt = cv2.VideoWriter_fourcc('m', 'p', '4', 'v')
    # from test
    image_size_4 = np.array([3840, 2160])  # 4K
    image_size_5 = np.array([5120, 2880])  # 5K
    image_size_53 = np.array([5312, 2988])  # 5.3K
    image_size_53_87 = np.array([5312, 4648])  # 5.3K8:7
    image_size_8 = np.array([7680, 4320])  # 8K
    image_size_10 = np.array([9600, 5400])  # 10K
    image_size_18 = np.array([17280, 17280])  # 18K

    focal_length_4 = np.array([1772.9, 1771.0])  #
    focal_length_5 = np.array([2216.13, 2217.5])  #
    focal_length_53_87 = np.array([2357.1, 2356.1])
    focal_length_8 = np.array([3545.8, 3542.0])  #
    focal_length_10 = np.array([4432.3, 4435.0])  #
    focal_length_18 = np.array([8085.6, 8087.4])

    principal_point_4 = np.array([1920.0, 1080.0])
    principal_point_5 = np.array([2560.0, 1440.0])
    principal_point_53_87 = np.array([2676.8, 2319.6])
    principal_point_8 = np.array([3840.0, 2160.0])
    principal_point_10 = np.array([4800.0, 2700.0])
    principal_point_18 = np.array([8640.0, 8640.0])

    width_4, height_4 = image_size_4[0], image_size_4[1]
    width_5, height_5 = image_size_5[0], image_size_5[1]
    width_53, height_53 = image_size_53[0], image_size_53[1]
    width_53_87, height_53_87 = image_size_53_87[0], image_size_53_87[1]  # 5312,4648
    width_8, height_8 = image_size_8[0], image_size_8[1]
    width_10, height_10 = image_size_10[0], image_size_10[1]
    width_18, height_18 = image_size_18[0], image_size_18[1]

    height_border_size_10_5 = int((height_10 - height_5) / 2)  # 1260
    width_border_size_10_5 = int((width_10 - width_5) / 2)  # 2240
    height_border_size_10_4 = int((height_10 - height_4) / 2)
    width_border_size_10_4 = int((width_10 - width_4) / 2)
    height_border_size_5_4 = int((height_5 - height_4) / 2)  # 2880-2160=720 720/2=360
    width_border_size_5_4 = int((width_5 - width_4) / 2)  # 5120-3840=1280 1280/2=640
    height_border_size_8_4 = int((height_8 - height_4) / 2)
    width_border_size_8_4 = int((width_8 - width_4) / 2)
    height_border_size_18_53 = int((height_18 - height_53) / 2)
    width_border_size_18_53 = int((width_18 - width_53) / 2)
    height_border_size_18_53_87 = int((height_18 - height_53_87) / 2)
    width_border_size_18_53_87 = int((width_18 - width_53_87) / 2)

    # 動画の書き込み設定 A,B,C,D
    #writer_pers = cv2.VideoWriter(file_name + '_stabilized.mp4', fmt, fps, (width_53, height_53))
    writer_pers = cv2.VideoWriter(path_file + '_stabilized.mp4', fmt, fps, (width_53, height_53))
    distortion_coefficients_53_87 = np.array([0.0313955397350478, 0.0597411012269733, -0.0433668199103408,
                                              0.0099241392094094])  # for GoPro12 5.3Kwide8:7 Gyroflow value
    distortion_coefficients_inv = np.array([0.61961826, 0.79117213, -1.1209189,
                                            2.55945338])
    mtx_10 = np.array(
        [[focal_length_10[0], 0.0, principal_point_10[0]], [0.0, focal_length_10[1], principal_point_10[1]],
         [0., 0., 1.]])
    mtx_8 = np.array(
        [[focal_length_8[0], 0.0, principal_point_8[0]], [0.0, focal_length_8[1], principal_point_8[1]], [0., 0., 1.]])
    mtx_5 = np.array(
        [[focal_length_5[0], 0.0, principal_point_5[0]], [0.0, focal_length_5[1], principal_point_5[1]], [0., 0., 1.]])
    mtx_4 = np.array(
        [[focal_length_4[0], 0.0, principal_point_4[0]], [0.0, focal_length_4[1], principal_point_4[1]], [0., 0., 1.]])
    mtx_53_87 = np.array(
        [[focal_length_53_87[0], 0.0, principal_point_53_87[0]], [0.0, focal_length_53_87[1], principal_point_53_87[1]],
         [0., 0., 1.]])
    mtx_18 = np.array(
        [[focal_length_18[0], 0.0, principal_point_18[0]], [0.0, focal_length_18[1], principal_point_18[1]],
         [0., 0., 1.]])
    mtx_5_10 = np.array(
        [[focal_length_5[0], 0.0, principal_point_10[0]], [0.0, focal_length_5[1], principal_point_10[1]],
         [0., 0., 1.]])
    mtx_4_8 = np.array(
        [[focal_length_4[0], 0.0, principal_point_8[0]], [0.0, focal_length_4[1], principal_point_8[1]], [0., 0., 1.]])
    mtx_53_87_18 = np.array(
        [[focal_length_53_87[0], 0.0, principal_point_18[0]], [0.0, focal_length_53_87[1], principal_point_18[1]],
         [0., 0., 1.]])
    newcameramtx_53_87_18 = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(mtx_53_87_18,
                                                                                   distortion_coefficients_53_87,
                                                                                   (width_18, height_18), None,
                                                                                   balance=1.0)
    newcameramtx_53_87 = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(mtx_53_87,
                                                                                distortion_coefficients_53_87,
                                                                                (width_53_87, height_53_87), None,
                                                                                balance=1.0)
    newcameramtx_53_87_18 = mtx_53_87_18
    mapcv1_53_87_18, mapcv2_53_87_18 = cv2.fisheye.initUndistortRectifyMap(mtx_53_87_18, distortion_coefficients_53_87,
                                                                           None, newcameramtx_53_87_18,
                                                                           (width_18, height_18), cv2.CV_16SC2)
    newcameramtx_53_87_18_redist = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(mtx_53_87_18,
                                                                                          distortion_coefficients_inv,
                                                                                          (width_18, height_18), None,
                                                                                          balance=1.0)
    mapcv1_53_87_18_redist, mapcv2_53_87_18_redist = cv2.fisheye.initUndistortRectifyMap(mtx_53_87_18,
                                                                                         distortion_coefficients_inv,
                                                                                         None,
                                                                                         newcameramtx_53_87_18_redist,
                                                                                         (width_18, height_18),
                                                                                         cv2.CV_16SC2)
    distortion_coefficients_inv_1 = np.zeros(4)
    for i in range(len(dist_para)):
        if dist_para[i] != 1.0:
            distortion_coefficients_inv_1 = distortion_coefficients_inv * dist_para[i]
    mapcv1_53_87_18_redist_1, mapcv2_53_87_18_redist_1 = cv2.fisheye.initUndistortRectifyMap(mtx_53_87_18,
                                                                                             distortion_coefficients_inv_1,
                                                                                             None,
                                                                                             newcameramtx_53_87_18_redist,
                                                                                             (width_18, height_18),
                                                                                             cv2.CV_16SC2)
    focal_x, focal_y = focal_length_53_87[0], focal_length_53_87[1]
    center_x, center_y = principal_point_18[0], principal_point_18[1]
    width, height = width_18, height_18

    trans_matrix = np.array([[0, -1, 0], [0, 0, -1], [1, 0, 0]])  # <A> good オリジナル 四元数要素ｙ<=>z
    K = np.array([[focal_x, 0., 0.], [0., focal_y, 0.], [0., 0., 1.]])
    K = K @ trans_matrix
    # センター値は個別に処理
    center_xy = np.array([center_x, center_y, 0.0]).T  # 3x1

    # ピクセル座標4点の入力
    up_left_i = np.array([-center_x, -center_y, 1.0]).T
    up_right_i = np.array([-center_x + width, -center_y, 1.0]).T
    down_left_i = np.array([-center_x, -center_y + height, 1.0]).T
    down_right_i = np.array([-center_x + width, -center_y + height, 1.0]).T

    up_left_s = up_left_i
    up_right_s = up_right_i
    down_left_s = down_left_i
    down_right_s = down_right_i

    up_left_s = up_left_s + center_xy
    up_right_s = up_right_s + center_xy
    down_left_s = down_left_s + center_xy
    down_right_s = down_right_s + center_xy

    lenloop = min(frame_count, len(quat_crop))

    for i in range(lenloop - 1):
        ret, src = cap.read()
        if not ret:
            break
        print("\r" + str(i), end="")
        #if i == 100:
        #    plt.imshow(cv2.cvtColor(src, cv2.COLOR_BGR2RGB))
        #    plt.show()
        src_1 = add_black_border(src, height_border_size_18_53_87, width_border_size_18_53_87)
        #if i == 100:
        #    plt.imshow(cv2.cvtColor(src_1, cv2.COLOR_BGR2RGB))
        #    plt.show()
        dst = cv2.remap(src_1, mapcv1_53_87_18, mapcv2_53_87_18, cv2.INTER_CUBIC, cv2.BORDER_TRANSPARENT)
        #if i == 100:
        #    plt.imshow(cv2.cvtColor(dst, cv2.COLOR_BGR2RGB))
        #    plt.show()
        quat_j = quat_crop[i]
        # 軸変換
        quat = np.quaternion(quat_j.w, quat_j.z, -quat_j.x, -quat_j.y)
        quat_j = np.normalized(quat)
        # 四元数⇒回転行列
        rot_matrix_j = quaternion.as_rotation_matrix(quat_j)
        # KRK^(-1)
        matrix_warp = K @ rot_matrix_j @ np.linalg.inv(K)
        # 4点のピクセル座標の変換
        up_left_d = matrix_warp @ up_left_i
        up_right_d = matrix_warp @ up_right_i
        down_left_d = matrix_warp @ down_left_i
        down_right_d = matrix_warp @ down_right_i

        up_left_k = up_left_d[2]
        up_right_k = up_right_d[2]
        down_left_k = down_left_d[2]
        down_right_k = down_right_d[2]

        up_left_d /= up_left_k
        up_right_d /= up_right_k
        down_left_d /= down_left_k
        down_right_d /= down_right_k

        up_left_d += center_xy
        up_right_d += center_xy
        down_left_d += center_xy
        down_right_d += center_xy

        # 変換前後でのピクセル座標
        src_pts = np.array(
            [[up_left_s[0], up_left_s[1]], [down_left_s[0], down_left_s[1]], [down_right_s[0], down_right_s[1]],
             [up_right_s[0], up_right_s[1]]], dtype=np.float32)
        dst_pts = np.array(
            [[up_left_d[0], up_left_d[1]], [down_left_d[0], down_left_d[1]], [down_right_d[0], down_right_d[1]],
             [up_right_d[0], up_right_d[1]]], dtype=np.float32)
        # 透視変換用行列の生成
        mat_trans = cv2.getPerspectiveTransform(src_pts, dst_pts)
        perspective_img = cv2.warpPerspective(dst, mat_trans, (width_18, height_18))
        #if i == 100:
        #    plt.imshow(cv2.cvtColor(perspective_img, cv2.COLOR_BGR2RGB))
        #    plt.show()
        if dist_para[i] == 1.0:
            dst_inv = cv2.remap(perspective_img, mapcv1_53_87_18_redist, mapcv2_53_87_18_redist, cv2.INTER_CUBIC,
                                cv2.BORDER_TRANSPARENT)
        else:
            # dst_inv= perspective_img
            dst_inv = cv2.remap(perspective_img, mapcv1_53_87_18_redist_1, mapcv2_53_87_18_redist_1, cv2.INTER_CUBIC,
                                cv2.BORDER_TRANSPARENT)
        #if i == 100:
        #    plt.imshow(cv2.cvtColor(dst_inv, cv2.COLOR_BGR2RGB))
        #    plt.show()
        cropped_img = dst_inv[height_border_size_18_53:height_border_size_18_53 + height_53,
                      width_border_size_18_53_87:width_border_size_18_53_87 + width_53_87, :]
        #if i == 100:
        #    plt.imshow(cv2.cvtColor(cropped_img, cv2.COLOR_BGR2RGB))
        #    plt.show()
        writer_pers.write(cropped_img)
        yield i+1
    cap.release()
    writer_pers.release()
    cv2.destroyAllWindows()
    #return # return と yield の併用はできない