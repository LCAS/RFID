
#!/usr/bin/env python

import numpy as np
import pandas as pd
import numpy.linalg as linalg
import pybayes, pybayes.pdfs, pybayes.filters
import frequencyModel

'''
Check  https://github.com/tridge/pyUblox/blob/master/pr_particle.py
for inspiration


- State is relative tag  to antena position: 
xt = [x y a]

- Measurement (observation) is what reader returns us:
yt = [rssi phi]

Reader also returns f, but that's not an observation. I follows a pseudo random sequence.
It's more a state element we already know.
Our idea is that is independant (somehow) between them. 
We model it as several independant filters.

- Conditional probability density function (CPdf) of state in t given state in t-1
p(x_t|x_{t-1}) 

Update model. Well, we assume tags are not moving, but robot is. So relative position changes...
I will use a general gaussian cpdf...

p_xt_xtp = pybayes.pdfs.GaussCPdf 
 
- Conditional probability density function (CPdf) of observation in t given state in t 
p(y_t|x_t)  

This is our observation model. Should be built from gathered data

p_yt_xt = pybayes.pdfs.GaussCPdf 


'''


def getObservation(reader):
    ''' 
    Gets last tag reading data: rssi and phi
    '''
    (yt,f) = reader.TODO!
    
    return (np.array(yt),f)



def main():
    # PARAMS .........................................................
    
    # load models
    modelURI = '/home/manolofc/catkin_ws/src/RFID/src/clients/ros/pr_model/tests/long/long_1tag_model.csv'
    models = pd.read_csv(modelURI)


    # frequency info
    freqVector = np.sort(np.unique(models['freq_khz'].copy()))
    minFreq = freqVector.min()
    maxFreq = freqVector.max()
    freqStep = freqVector[1]-freqVector[0] # we assume they are equispaced


    # num of particles
    n = 1000
    
    # initial state guess for tag
    x0 = TODO!
    y0 = TODO!
    a0 = TODO!
    x0_cov = TODO!
    y0_cov = TODO!
    a0_cov = TODO!

    
    # ................................................................
    mean0 = np.array([x0,y0,a0])
    cov0 = np.diag([x0_cov, y0_cov, a0_cov])

    # create one particle filter per frequency
    pfilt = []
    for fi in freqVector:
        statistics_f = models[models['freq_khz']==fi].copy()
        pfilti = frequencyModel.myFpFFilter(n, statistics_f,mean0,cov0)
        pfilt.append(pfilti)
        
   
    while True:
        # get data
        (yt,f) = getObservation(reader)
        f_index = freqVector.tolist().index(f)

        # perform bayes rule on new observation on corresponding filter
        pfilt[f_index].addObservation(yt)

        # view best guessess
        print(pfilt[f_index].mean(), pfilt[f_index].variance())
        
        
if __name__ == "__main__":
    main()
