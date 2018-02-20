
#!/usr/bin/env python

import numpy as np
import numpy.linalg as linalg
import pybayes, pybayes.pdfs, pybayes.filters

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

def modelDataMean(xt):
    (xi, yi, ai) = xt
    subSet =    readings[ (readings['rel_x_m']<=xi)     & (readings['rel_x_m']>xi_prev)     &
                      (readings['rel_y_m']<=yi)     & (readings['rel_y_m']>yi_prev)     & 
                      (readings['rel_yaw_rad']<=ai) & (readings['rel_yaw_rad']>ai_prev) & 
                      (readings['freq_khz']<=fi)    & (readings['freq_khz']>fi_prev)     ]

    
    return yt

def sensor_model_mu_f(xt):
    ''' Return mean of Gaussian PDF for measurement yt (reader observation) given xt (tag pose).  
        Mean is at the ideal measurement'''
 
    yt = modelDataMean(xt)
    
    return numpy.array(yt)

def sensor_model_R_f(xt):
    ''' Return covariance matric of Gaussian PDF for measurement yt (reader observation) given xt (tag pose).  
    '''

    yt_cov_Matrix = modelDataCov(xt) TODO!!!!

    return yt_cov_Matrix


def update_model_mu_f(xtp):
    '''Return mean of Gaussian PDF for state xt given x(t-1).  
    Assume relatively slow robot motion (compared with readings), mean at old state'''
    return xtp

def update_model_R_f(xtp):
    '''Return covariance of Gaussian PDF for state xt given x(t-1).
    How random we consider this?'''
    
    # TODO! this is a randomly chosen number....
    state_cov = 10 

    return numpy.diag([state_cov, state_cov, state_cov/10])

def getObservation(reader):
    ''' 
    Gets last tag reading data: rssi and phi
    '''
    (yt,f) = rfid.TODO!
    
    return (np.array(yt),f)



def build_init_pdf(init_mean,init_cov):
    '''
    This builds the pdf used to generate random states for particles
    '''
    i_pdf = pybayes.pdfs.GaussPdf(init_mean, init_cov)
    
    return i_pdf
    
def build_update_model():
    '''
    Returns Conditional probability of state in t given state in t-1
    We assume tags are not moving, but robot is. So relative position changes...
    I will use a general gaussian cpdf...
    '''
    
    # random variable (state) dimension. Occurs to be the same than condition
    len_xt = 3 
    
    p_xt_xtp = pybayes.pdfs.GaussCPdf(len_xt, len_xt, update_model_mu_f, update_model_R_f)
    
    return p_xt_xtp

def build_sensor_model():
    '''
    Returns Conditional probability of observation in t given state in t. 
    This is our observation model. Should be built from gathered data
    I will use a general gaussian cpdf...
    '''
    # random variable (state) dimension. Occurs to be the same than condition
    len_xt = 3 
    len_yt = 3
    
    p_yt_xt  = pybayes.pdfs.GaussCPdf(len_yt, sensor_model_mu_f, sensor_model_R_f)
        
    return p_yt_xt

def castFtoIndex(f,minFreq,maxFreq,freqStep):
    f_index = (int) ((f - minFreq)/freqStep)
    return f_index

def main():
    # PARAMS .........................................................
    
    # frequency info
    minFreq = 900
    maxFreq = 940
    freqStep = 20
    maxFIndex = astFtoIndex(maxFreq,minFreq,maxFreq,freqStep)
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
    
    #where our observations comes from
    rfid_reader = TODO!
    
    init_pdf = build_init_pdf(mean0,cov0)
    p_xt_xtp = build_update_model()
    p_yt_xt = build_sensor_model()

    # create one particle filter per frequency
    pfilt = []
    for i in range(0,1+maxFIndex):
        pfilti = pybayes.filters.ParticleFilter(n, init_pdf, p_xt_xtp, p_yt_xt)
        pfilt.append(pfilti)
        
   
    while True:
        # get data
        (yt,f) = getObservation(reader)
        f_index = castFtoIndex(f)
        
        # perform bayes rule on new observation on corresponding filter
        pfilt[f_index].bayes(yt)
        # view best guessess
        print(pfilt[f_index].posterior().mean(), pfilt[f_index].posterior().variance())
        
        
        
if __name__ == "__main__":
    main()
