#!/usr/bin/env python

from matplotlib.backends.backend_pdf import PdfPages
import numpy as np
import sys
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

import math
import pickle

import scipy.ndimage

import matplotlib.colors as colors
import os
from nav_msgs.msg import  OccupancyGrid
from matplotlib.colors import BoundaryNorm
from matplotlib.ticker import MaxNLocator

import scipy.ndimage


# Main function.
def avRssi():
    gridSize = 9
    gridResolution = 0.3
    transmissionPower = 3000



    #print os.getcwd()
    try:
        fname = "../models/" + str(transmissionPower) + ".p"
        file = (open(fname, "rb"))
    except IOError:
        fname = "scripts/models/" + str(transmissionPower) + ".p"
        file = (open(fname, "rb"))

    modelList = pickle.load(file)
    modelList = sorted(modelList, key=lambda m: m[1])

    # pick the desired colormap, sensible levels, and define a normalization
    # instance which takes data values and translates those into levels.
    cmap = plt.get_cmap('OrRd')

    i = 0
    X, Y = np.mgrid[slice(-(gridSize ), (gridSize ), gridResolution),
                    slice(-(gridSize ), (gridSize ), gridResolution)]

    minX=-3.0
    maxX=3.0
    minY=-3.0
    maxY=3.0
    sig = 0.45
    thres=-10
    saveME=False

    # for m in modelList[:-1:]:
    #     figt, axt = plt.subplots(nrows=1, ncols=1)
    #     #fig2t, ax2t = plt.subplots(nrows=1, ncols=1)
    #     (cf, f)    = plotAModelThres (m, axt,  X, Y, True, minX, maxX, minY, maxY, '', sig,thres)
    #
    #     plt.show()
    #     if saveME:
    #         pdf = PdfPages('./figures/AvRSSI_cart_' + str(f) + '.pdf')
    #         pdf.savefig(figt)
    #         pdf.close()
    #
    #     # pdf = PdfPages('./figures/AvRSSI_pol_' + str(f) + '.pdf')
    #     # pdf.savefig(fig2t)
    #     # pdf.close()



    ##################

    for m in modelList[:-1:]:
            width = m[2].info.width
            height = m[2].info.height
            avRssi = np.array(m[4]).reshape(width, height)
            detections = np.array(m[3]).reshape(width, height)
            avRssi = avRssi.astype(float)
            avRssi[detections == 0] = -1006
            avRssi=scipy.ndimage.filters.gaussian_filter(avRssi, sigma=sig, mode='reflect')
            try:
                total2 = np.append(avRssi[np.newaxis, :], total2, axis=0)
            except:
                total2=np.array(avRssi[np.newaxis, :])

    indices = np.ones(total2.shape)
    indices[np.isnan(total2)]=0
    print '.....'
    print indices.sum(axis=0)
    for i in range(0,50):
        print total2[i,:,:]

    avRssi = np.average(total2,axis=0,weights=indices)

    reescaledAVR = scipy.ndimage.filters.gaussian_filter(avRssi, sigma=sig, mode='reflect')

    reescaledAVR = (reescaledAVR + np.flipud(reescaledAVR)) / 2
    reescaledAVR = np.rot90(np.flipud(reescaledAVR), 3)



    showAxis=True
    title='RSSI over 70%'
    fig, ax = plt.subplots(nrows=1, ncols=1)

    th1 = 49

    print 'max'
    print reescaledAVR.max()
    print 'mean'

    print reescaledAVR.mean()
    print 'min'

    print reescaledAVR.min()

    reescaledAVR[reescaledAVR <= -120] = 0


    cf = ax.pcolormesh(X, Y, reescaledAVR, cmap=plt.get_cmap('bone') )
    #                      norm=colors.LogNorm(vmin=1, vmax=realMax - realMin + 1),
    #                      cmap=cmap,shading='none')
    ax.plot(0, 0, "w>", markersize=8,label='Robot Pose')
    ax.legend(loc='upper center', shadow=True)

    ax.get_xaxis().set_visible(showAxis)
    ax.get_yaxis().set_visible(showAxis)
    ax.set_xlim((minX, maxX))
    ax.set_ylim((minY, maxY))
    ax.set_aspect('equal')
    ax.set_xlabel('X (m.)')
    ax.set_ylabel('Y (m.)')
    #ax.set_title(title)


    plt.colorbar(cf, ax=ax)

    plt.show()


def swapRSSI():
    gridSize = 9
    gridResolution = 0.3
    transmissionPower = 3000



    #print os.getcwd()
    try:
        fname = "../models/" + str(transmissionPower) + ".p"
        file = (open(fname, "rb"))
    except IOError:
        fname = "scripts/models/" + str(transmissionPower) + ".p"
        file = (open(fname, "rb"))

    modelList = pickle.load(file)
    modelList = sorted(modelList, key=lambda m: m[1])

    # pick the desired colormap, sensible levels, and define a normalization
    # instance which takes data values and translates those into levels.
    cmap = plt.get_cmap('OrRd')

    i = 0
    X, Y = np.mgrid[slice(-(gridSize ), (gridSize ), gridResolution),
                    slice(-(gridSize ), (gridSize ), gridResolution)]

    minX=-2.0
    maxX=2.0
    minY=-3.0
    maxY=3.0
    sig = 0.45

    m = modelList[0]
    width  = m[2].info.width
    height = m[2].info.height
    counter = np.zeros((width, height))

    avRssi = np.array(m[4]).reshape(width, height)
    detections = np.array(m[3]).reshape(width, height)

    avRssi = avRssi.astype(float)

    avRssi[detections == 0] = -99


    reescaledRSSI = scipy.ndimage.filters.gaussian_filter(avRssi, sigma=sig, mode='reflect')
    reescaledRSSI = (reescaledRSSI + np.flipud(reescaledRSSI)) / 2
    reescaledRSSI = np.rot90(np.flipud(reescaledRSSI), 3)


    showAxis=True
    title=''
    fig, ax = plt.subplots(nrows=1, ncols=1)


    print reescaledRSSI.max()
    print reescaledRSSI.mean()
    print reescaledRSSI.min()

    times=7

    ax.plot(0, 0, "w>", markersize=8, label='Robot Pose')
    ax.legend(loc='upper center', shadow=True)
    total = np.array(reescaledRSSI)

    plt.ion()
    title = ' average power'
    for times in range(670,700,1):

        #total[total >= th1] = 255
        #total[total < th1] = 0

        cf = ax.pcolormesh(X, Y, total, cmap=plt.get_cmap('bone'))
        #                      norm=colors.LogNorm(vmin=1, vmax=realMax - realMin + 1),
        #                      cmap=cmap,shading='none')


        ax.get_xaxis().set_visible(showAxis)
        ax.get_yaxis().set_visible(showAxis)
        ax.set_xlim((minX, maxX))
        ax.set_ylim((minY, maxY))
        ax.set_aspect('equal')
        ax.set_xlabel('X (m.)')
        ax.set_ylabel('Y (m.)')
        ax.set_title(title)

        plt.pause(0.01)
        sys.stdin.read(1)
        total = np.array(reescaledRSSI)

        th1 = times*total.mean()/1000
        th2 = (times-1) * total.mean() / 1000
        title = str(times/1000.0) + ' times the average'

        print times/1000.0
        print th1
        print th2

        #total[reescaledRSSI < th1] = 1
        #total[(reescaledRSSI >= th1) & (reescaledRSSI < th2)] = 0.5
        #total[ (reescaledRSSI > th2)] = 0
        total[(reescaledRSSI < th2)] = 0
        print (total)



##################################



def plotRSSI(threshold=-64.4):
    gridSize = 9
    gridResolution = 0.3
    transmissionPower = 3000


    #
    #print os.getcwd()
    try:
        fname = "../models/" + str(transmissionPower) + ".p"
        file = (open(fname, "rb"))
    except IOError:
        fname = "scripts/models/" + str(transmissionPower) + ".p"
        file = (open(fname, "rb"))

    modelList = pickle.load(file)
    modelList = sorted(modelList, key=lambda m: m[1])

    # pick the desired colormap, sensible levels, and define a normalization
    # instance which takes data values and translates those into levels.
    cmap = plt.get_cmap('OrRd')

    i = 0
    X, Y = np.mgrid[slice(-(gridSize ), (gridSize ), gridResolution),
                    slice(-(gridSize ), (gridSize ), gridResolution)]

    minX = -0.9
    maxX = 2.4
    minY = -1.8
    maxY = 1.8
    sig = 0.45

    m = modelList[0]
    width  = m[2].info.width
    height = m[2].info.height

    title=''

    avRssi = np.array(m[4]).reshape(width, height)
    detections = np.array(m[3]).reshape(width, height)

    avRssi = avRssi.astype(float)

    avRssi[detections == 0] = -99


    reescaledRSSI = scipy.ndimage.filters.gaussian_filter(avRssi, sigma=sig, mode='reflect')
    reescaledRSSI = (reescaledRSSI + np.flipud(reescaledRSSI)) / 2
    reescaledRSSI = np.rot90(np.flipud(reescaledRSSI), 3)


    showAxis=True
    #fig, ax = plt.subplots(nrows=1, ncols=1)

    gs = gridspec.GridSpec(1, 2,
                           width_ratios=[10, 1],
                           )
    ax = plt.subplot(gs[0])
    ax2 = plt.subplot(gs[1])


    total = np.array(reescaledRSSI)

    total[(reescaledRSSI < threshold)] = 0



    ax.plot(0, 0, "w>", markersize=8, label='Robot Pose')
    #ax.legend(loc='upper center', shadow=True)



    cf = ax.pcolormesh(X, Y, total, cmap=plt.get_cmap('bone'),edgecolor='grey',linewidth=0.01)
    #                      norm=colors.LogNorm(vmin=1, vmax=realMax - realMin + 1),
    #                      cmap=cmap,shading='none')
    cbar=plt.colorbar(cf, cax=ax2,ticks=[0, -8, -16, -24, -32, -40, -48, -56, -64])
    cbar.ax.set_yticklabels(['< -40 dB', '-8', '-16', '-24', '-32', '-40', '-48', '-56', '-64'])

    #ax.legend(loc='upper center', shadow=True)
    ax.get_xaxis().set_visible(showAxis)
    ax.get_yaxis().set_visible(showAxis)
    ax.set_xlim((minX, maxX))
    ax.set_ylim((minY, maxY))
    ax.set_aspect('equal')
    ax.set_xlabel('X (m.)')
    ax.set_ylabel('Y (m.)')
    ax.set_title(title)
    plt.show()


##################################
    #th1=    0.527991571385 #47
    #th2=   0.19  #18

def plotDETECT(mid=0.19,high=0.54):
        # Main function.
        gridSize = 9
        gridResolution = 0.3
        transmissionPower = 3000

        # print os.getcwd()
        try:
            fname = "../models/" + str(transmissionPower) + ".p"
            file = (open(fname, "rb"))
        except IOError:
            fname = "scripts/models/" + str(transmissionPower) + ".p"
            file = (open(fname, "rb"))

        modelList = pickle.load(file)
        modelList = sorted(modelList, key=lambda m: m[1])

        # pick the desired colormap, sensible levels, and define a normalization
        # instance which takes data values and translates those into levels.
        cmap = plt.get_cmap('OrRd')

        i = 0
        X, Y = np.mgrid[slice(-(gridSize), (gridSize), gridResolution),
                        slice(-(gridSize), (gridSize), gridResolution)]

        minX = -0.9
        maxX = 2.4
        minY = -1.8
        maxY = 1.8
        sig = 1

        for m in modelList[:-1:]:
            width = m[2].info.width
            height = m[2].info.height

            detections = np.array(m[3]).reshape(width, height)

            detections = detections.astype(float)

            reescaledDetec = scipy.ndimage.filters.gaussian_filter(detections, sigma=sig, mode='reflect')

            reescaledDetec = (reescaledDetec + np.flipud(reescaledDetec)) / 2
            reescaledDetec = np.rot90(np.flipud(reescaledDetec), 3)

            try:
                total = total + reescaledDetec
            except:
                total = reescaledDetec

        maxDetect = np.nanmax(np.nanmax(total))

        total = np.array(total/maxDetect)
        total0 = np.array(total)
        showAxis = True
        title = ''

        gs = gridspec.GridSpec(1, 2,
                               width_ratios=[10, 1],
                               )
        ax = plt.subplot(gs[0])
        ax2 = plt.subplot(gs[1])


        ax.plot(0, 0, "w>", markersize=8, label='Robot Pose')
        #ax.legend(loc='upper center', shadow=True)
        ax.grid(True)

        th2 = mid
        th1 = high

        total[(total0 < th2)] = 1
        total[total0 > th1] = 0
        total[(total0 <= th1) & (total0 > th2)] = 0.5

        cf = ax.pcolormesh(X, Y, total, cmap=plt.get_cmap('bone'),edgecolor='grey',linewidth=0.01)

        cbar = plt.colorbar(cf, cax=ax2, ticks=[0, 0.5,1])
        cbar.ax.set_yticklabels(['>75 %', '30-75%', '< 30 %'])

        ax.get_xaxis().set_visible(showAxis)
        ax.get_yaxis().set_visible(showAxis)
        ax.set_xlim((minX, maxX))
        ax.set_ylim((minY, maxY))
        ax.set_aspect('equal')
        ax.set_xlabel('X (m.)')
        ax.set_ylabel('Y (m.)')
        plt.show()

##################################


def plotDETECTi(i,mid=0.064,high=0.165):
        # Main function.
        gridSize = 9
        gridResolution = 0.3
        transmissionPower = 3000

        # print os.getcwd()
        try:
            fname = "../models/" + str(transmissionPower) + ".p"
            file = (open(fname, "rb"))
        except IOError:
            fname = "scripts/models/" + str(transmissionPower) + ".p"
            file = (open(fname, "rb"))

        modelList = pickle.load(file)
        modelList = sorted(modelList, key=lambda m: m[1])

        # pick the desired colormap, sensible levels, and define a normalization
        # instance which takes data values and translates those into levels.
        cmap = plt.get_cmap('OrRd')


        X, Y = np.mgrid[slice(-(gridSize), (gridSize), gridResolution),
                        slice(-(gridSize), (gridSize), gridResolution)]

        minX = -0.9
        maxX = 2.4
        minY = -1.8
        maxY = 1.8
        sig = 0.45

        m = modelList[i]
        width = m[2].info.width
        height = m[2].info.height

        detections = np.array(m[3]).reshape(width, height)

        detections = detections.astype(float)

        reescaledDetec = scipy.ndimage.filters.gaussian_filter(detections, sigma=sig, mode='reflect')

        reescaledDetec = (reescaledDetec + np.flipud(reescaledDetec)) / 2
        reescaledDetec = np.rot90(np.flipud(reescaledDetec), 3)

        total = reescaledDetec

        maxDetect = np.nanmax(np.nanmax(total))

        total = np.array(total/maxDetect)
        total0 = np.array(total)
        showAxis = True
        title = ''


        fig, ax = plt.subplots(nrows=1, ncols=1)


        ax.plot(0, 0, "w>", markersize=8, label='Robot Pose')
        #ax.legend(loc='upper center', shadow=True)
        ax.grid(True)

        th2 = mid
        th1 = high

        total[(total0 < th2)] = 1
        total[total0 > th1] = 0
        total[(total0 <= th1) & (total0 > th2)] = 0.5

        cf = ax.pcolormesh(X, Y, total, cmap=plt.get_cmap('bone'),edgecolor='grey',linewidth=0.01)



        ax.get_xaxis().set_visible(showAxis)
        ax.get_yaxis().set_visible(showAxis)
        ax.set_xlim((minX, maxX))
        ax.set_ylim((minY, maxY))
        ax.set_aspect('equal')
        ax.set_xlabel('X (m.)')
        ax.set_ylabel('Y (m.)')
        plt.show()

##################################


def swapDETECT():
        # Main function.
        gridSize = 9
        gridResolution = 0.3
        transmissionPower = 3000

        # print os.getcwd()
        try:
            fname = "../models/" + str(transmissionPower) + ".p"
            file = (open(fname, "rb"))
        except IOError:
            fname = "scripts/models/" + str(transmissionPower) + ".p"
            file = (open(fname, "rb"))

        modelList = pickle.load(file)
        modelList = sorted(modelList, key=lambda m: m[1])

        # pick the desired colormap, sensible levels, and define a normalization
        # instance which takes data values and translates those into levels.
        cmap = plt.get_cmap('OrRd')

        i = 0
        X, Y = np.mgrid[slice(-(gridSize), (gridSize), gridResolution),
                        slice(-(gridSize), (gridSize), gridResolution)]

        minX = -0.9
        maxX = 2.4
        minY = -1.8
        maxY = 1.8
        sig = 1

        for m in modelList[:-1:]:
            width = m[2].info.width
            height = m[2].info.height

            detections = np.array(m[3]).reshape(width, height)

            detections = detections.astype(float)

            reescaledDetec = scipy.ndimage.filters.gaussian_filter(detections, sigma=sig, mode='reflect')

            reescaledDetec = (reescaledDetec + np.flipud(reescaledDetec)) / 2
            reescaledDetec = np.rot90(np.flipud(reescaledDetec), 3)

            try:
                total = total + reescaledDetec
            except:
                total = reescaledDetec

        maxDetect = np.nanmax(np.nanmax(total))

        total = np.array(total/maxDetect)
        total0 = np.array(total)
        showAxis = True
        title = ''


        fig, ax = plt.subplots(nrows=1, ncols=1)

        #print total.max()
        #print total.mean()
        #print total.min()

        gs = gridspec.GridSpec(1, 2,
                               width_ratios=[10, 1],
                               )

        ax = plt.subplot(gs[0])
        ax2 = plt.subplot(gs[1])

        ax.plot(0, 0, "w>", markersize=8, label='Robot Pose')
        ax.legend(loc='upper center', shadow=True)
        ax.grid(True)
        plt.ion()
        for times in range(15, 70, 1):
            title = 'Prob. detection Above 95% (black) - 85% (gray)'

            # total[total >= th1] = 255
            # total[total < th1] = 0

            cf = ax.pcolormesh(X, Y, total, cmap=plt.get_cmap('bone'),edgecolor='grey',linewidth=0.01)
            #                      norm=colors.LogNorm(vmin=1, vmax=realMax - realMin + 1),
            #                      cmap=cmap,shading='none')


            plt.colorbar(cf, cax=ax2)
            ax.get_xaxis().set_visible(showAxis)
            ax.get_yaxis().set_visible(showAxis)
            ax.set_xlim((minX, maxX))
            ax.set_ylim((minY, maxY))
            ax.set_aspect('equal')
            ax.set_xlabel('X (m.)')
            ax.set_ylabel('Y (m.)')
            ax.set_title(title)


            # cellSize=2.8
            # major_ticks = np.linspace(-2, 10, num=15)/cellSize
            # ax.set_xticks(major_ticks)
            #
            # major_ticksy = np.linspace(-6.4, 6.4, num=15)/cellSize
            # ax.set_yticks(major_ticksy)
            # ax.grid(True)


            plt.pause(0.01)
            sys.stdin.read(1)
            total = np.array(total0)

            th2 = 5 * total.mean()
            th1 = 13 * total.mean()

            th1 = times * total.mean() / 2
            th2 = (times - 1) * total.mean() / 2
            # th2 = th1
            print times
            print  th2
            print  th1

            #0.0633862417701
            #0.164804228602

            total[(total0 < th2)] = 1
            total[total0 > th1] = 0
            total[(total0 <= th1) & (total0 > th2)] = 0.5

            #print (total)
##################################


def swapDETECTi(i):
        # Main function.
        gridSize = 9
        gridResolution = 0.3
        transmissionPower = 3000

        # print os.getcwd()
        try:
            fname = "../models/" + str(transmissionPower) + ".p"
            file = (open(fname, "rb"))
        except IOError:
            fname = "scripts/models/" + str(transmissionPower) + ".p"
            file = (open(fname, "rb"))

        modelList = pickle.load(file)
        modelList = sorted(modelList, key=lambda m: m[1])

        # pick the desired colormap, sensible levels, and define a normalization
        # instance which takes data values and translates those into levels.
        cmap = plt.get_cmap('OrRd')


        X, Y = np.mgrid[slice(-(gridSize), (gridSize), gridResolution),
                        slice(-(gridSize), (gridSize), gridResolution)]

        minX = -0.9
        maxX = 2.4
        minY = -1.8
        maxY = 1.8
        sig = 1.0


        m = modelList[i]
        width = m[2].info.width
        height = m[2].info.height

        detections = np.array(m[3]).reshape(width, height)

        detections = detections.astype(float)

        reescaledDetec = scipy.ndimage.filters.gaussian_filter(detections, sigma=sig, mode='reflect')
        reescaledDetec = (reescaledDetec + np.flipud(reescaledDetec)) / 2
        reescaledDetec = np.rot90(np.flipud(reescaledDetec), 3)
        total = reescaledDetec

        maxDetect = np.nanmax(np.nanmax(total))

        total = np.array(total/maxDetect)
        total0 = np.array(total)
        showAxis = True
        title = ''


        fig, ax = plt.subplots(nrows=1, ncols=1)

        print total.max()
        print total.mean()
        print total.min()

        gs = gridspec.GridSpec(1, 2,
                               width_ratios=[10, 1],
                               )

        ax = plt.subplot(gs[0])
        ax2 = plt.subplot(gs[1])

        ax.plot(0, 0, "w>", markersize=8, label='Robot Pose')
        ax.legend(loc='upper center', shadow=True)
        ax.grid(True)
        plt.ion()
        for times in range(1, 30, 1):
            title = 'Prob. detection Above 95% (black) - 85% (gray)'

            # total[total >= th1] = 255
            # total[total < th1] = 0

            cf = ax.pcolormesh(X, Y, total, cmap=plt.get_cmap('bone'),edgecolor='grey',linewidth=0.01)
            #                      norm=colors.LogNorm(vmin=1, vmax=realMax - realMin + 1),
            #                      cmap=cmap,shading='none')


            plt.colorbar(cf, cax=ax2)
            ax.get_xaxis().set_visible(showAxis)
            ax.get_yaxis().set_visible(showAxis)
            ax.set_xlim((minX, maxX))
            ax.set_ylim((minY, maxY))
            ax.set_aspect('equal')
            ax.set_xlabel('X (m.)')
            ax.set_ylabel('Y (m.)')
            ax.set_title(title)


            # cellSize=2.8
            # major_ticks = np.linspace(-2, 10, num=15)/cellSize
            # ax.set_xticks(major_ticks)
            #
            # major_ticksy = np.linspace(-6.4, 6.4, num=15)/cellSize
            # ax.set_yticks(major_ticksy)
            # ax.grid(True)


            plt.pause(0.01)
            sys.stdin.read(1)
            total = np.array(total0)

            th2 = 5 * total.mean()
            th1 = 13 * total.mean()

            th1 = times * total.mean()
            th2 = (times - 1) * total.mean()
            # th2 = th1
            print  th2
            print  th1

            #0.0633862417701
            #0.164804228602

            total[(total0 < th2)] = 1
            total[total0 > th1] = 0
            total[(total0 <= th1) & (total0 > th2)] = 0.5

            #print (total)

# Main function.
if __name__ == '__main__':
    #plotRSSI()
    plotDETECT()
    #swapDETECT()
    #plotDETECTi(36)


