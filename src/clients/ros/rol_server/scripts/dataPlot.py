import pickle
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

def getDataFrame(fullPath,fileName,endTime):

    if fullPath[-1]!='/':
        fullPath=fullPath+'/'

    objectData=pickle.load(open(fullPath+fileName,'rw'))

    numPoints=len(objectData)
    locations=sorted(objectData[0].keys())
    locations.pop(locations.index('time'))
    # locations=['kitchen','lounge', 'bathroom corridor', 'entrance', 'entrance corridor','kitchen - coffee', 'kitchen - fridge', 'lounge - sofas','lounge - tv', 'time', 'workzone G', 'workzone P', 'workzone T','workzone Y', 'workzone corridor']

    df2 = pd.DataFrame(objectData)
    df2['seconds']=df2['time'] - df2['time'][0]
    df2['time']=pd.to_datetime(df2['time'],unit='s')
    df2.set_index('time',drop=False,append=False,inplace=True)
    df2.set_index('seconds',drop=False,append=False,inplace=True)
    df2.set_index('detects', drop=False, append=False, inplace=True)

    df2=df2[df2['seconds']<endTime]
    return df2

def purgueColumns(dataframe,unwanted):
    ans = dataframe
    for i in unwanted:
        try:
            del ans[i]
        except:
            pass
    return ans


def confidence(dataframe,trueLoc,keyList,centr):
    maxNames=[]
    index = 0
    maxVals=np.zeros(len(dataframe.index))
    dists=np.zeros(len(dataframe.index))

    trueRegionPos = np.array(centr[trueLoc])

    for i, row in dataframe.iterrows():
        maxName=trueLoc
        trueRegionConfidence=row[trueLoc]
        weightPos = trueRegionPos * trueRegionConfidence

        for key in keyList:
            regionConfidence=row[key]
            regionPos = np.array(centr[key])
            weightPos=weightPos + regionPos *regionConfidence

            diff=regionConfidence-trueRegionConfidence
            if diff>maxVals[index]:
                maxVals[index]=diff
                maxName=key
        maxNames.append(maxName)
        dists[index]= np.linalg.norm(trueRegionPos-weightPos)
        index=index+1


    return (maxNames,maxVals,dists)

def changeYaxisToPercent(ax):
    #change labels to show percents
    a = ax.get_yticks().tolist()
    for i in range(0,len(a)):
        a[i]=(str(100.0*a[i]))
    ax.set_yticklabels(a)
    #.........................

def allPlots(fullPath,endTime,minConf,deleteItems,fileName,trueLoc,centr):
    draw=True

    df = getDataFrame(fullPath, fileName, endTime)

    purgueColumns(df, deleteItems)

    # delete unprobable regions under 10 %
    meanConf = df.mean()
    # meanConf.sort_values(inplace=True)
    dropped = meanConf[meanConf < minConf].keys().tolist()
    locList = meanConf[meanConf > minConf].keys().tolist()
    locList.remove('seconds')
    locList.remove('detects')
    #print locList

    purgueColumns(df, dropped)

    locs, dife,dist = confidence(df, trueLoc,locList,centr)
    #print locs

    detect = df['detects']
    del df['detects']
    seconds = df['seconds']
    del df['seconds']


    print 'Object:',fileName[11:-2]
    print 'Av. Error (m.):', np.average(dist)

    locs=pd.DataFrame(locs)
    goodLocs=locs[locs==trueLoc].count().tolist()[0]
    totalLocs=locs.count().tolist()[0]

    print 'Region accuracy (% times):', (100.0*goodLocs/totalLocs)
    print '........................................................'

    ax=df.plot(x=seconds)
    changeYaxisToPercent(ax)
    ax.set_xlabel('Elapsed seconds')
    ax.set_ylabel('Region Confidence (%)')
    plt.legend(title="Regions",loc='best')
    if draw:
        plt.show()

    ax=df.plot(x=detect)
    changeYaxisToPercent(ax)
    ax.set_xlabel('Num. of Tag detections')
    ax.set_ylabel('Region Confidence (%)')
    plt.legend(title="Regions",loc='best')
    if draw:
        plt.show()

    ax=df.plot(kind='box')
    changeYaxisToPercent(ax)
    ax.set_xlabel('Region')
    ax.set_ylabel('Region Confidence (%)')
    if draw:
        plt.show()

    df['dife']=dife
    ax=df.plot(y='dife', x=seconds,legend=None)
    changeYaxisToPercent(ax)
    ax.set_xlabel('Elapsed seconds')
    #plt.legend(title="Regions",loc='best')
    ax.set_ylabel('Confidence error (%)')
    if draw:
        plt.show()


    df['dife'] = dist
    ax = df.plot(y='dife', x=seconds,legend=None)
    ax.set_xlabel('Elapsed seconds')
    ax.set_ylabel('Distance Error to region centroid (m.)')
    if draw:
        plt.show()




fullPath='/home/mfcarmona/catkin_ws/src/ENRICHME/codes/ais/LibMercuryRFID/src/clients/ros/rfid_grid_map/launch/article/3/'
endTime=120
minConf=0.1

centroids={'kitchen': (3.015,0.565),
'kitchen - fridge': (3.015,-0.275),
'kitchen - coffee': ( 3.015,1.300),
'entrance corridor': ( 0.910,3.605),
'bathroom corridor': ( 0.655,0.435),
'workzone corridor': ( 0.410,9.435),
'workzone G': ( -2.620,6.860),
'workzone P': ( 2.935,6.960),
'workzone Y': ( -2.555,11.270),
'workzone T': ( 3.120,11.345),
'entrance': ( -3.915,3.605),
'lounge': ( -2.700,0.470),
'lounge - sofas': ( -3.825,0.470),
'lounge - tv': (-1.395,0.470)}


# delete subregions - keep only regions
deleteItems = [ 'kitchen - coffee', 'kitchen - fridge', 'lounge - sofas', 'lounge - tv']
#, 'entrance', 'workzone G', 'workzone P', 'workzone T', 'workzone Y', 'workzone corridor']


#1 ...............................................................................
fileName='REPLAY_UOL_kitchen table.p'
trueLoc='lounge'
allPlots(fullPath,endTime,minConf,deleteItems,fileName,trueLoc,centroids)
#...............................................................................


#1 ...............................................................................
fileName='REPLAY_UOL_tape holder.p'
trueLoc='lounge'
allPlots(fullPath,endTime,minConf,deleteItems,fileName,trueLoc,centroids)
#...............................................................................


#1 ...............................................................................
fileName='REPLAY_UOL_lounge table.p'
trueLoc='lounge'
allPlots(fullPath,endTime,minConf,deleteItems,fileName,trueLoc,centroids)
#...............................................................................


exit()





objectData=pickle.load(open('/home/mfcarmona/catkin_ws/src/ENRICHME/codes/ais/LibMercuryRFID/src/clients/ros/rfid_grid_map/launch/article/2/REPLAY_UOL_red stapler.p','rw'))



exit()

# labels = [item.get_text() for item in ax.get_xticklabels()]
# labels[1] = 'Testing'
#
# ax.set_xticklabels(labels)
#
# plt.show()

'''
Results for FDG data file:
Not a clear idea of where are objects. Very short dataset:

wallet: living room, sofa

keys: living room, dining

pillbox: living room, dining

remote: living room, sofa

glasses: living room, dining table
'''
