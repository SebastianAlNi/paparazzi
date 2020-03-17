

import numpy as np
import extract_information_flow_field as OF
import matplotlib.pyplot as plt

 #if not os.path.exists('pics'): 
        
# Change the image numbers below to answer the questions:
kk=1;
dd = np.zeros((2200,1))  
cc = np.zeros((2201,1))  


for x in range(50,100):

    img_nr_1 = x;
    img_nr_2 = x+1;
    points_old, points_new, flow_vectors,c, d = OF.show_flow(x,img_nr_1, img_nr_2);
    #length=len(points_old)
  #  print (c)
  #  print (points_old)
  #  print(length)
    dd[kk,0]=d;
    cc[kk,0]=c; 
    kk=kk+1;
#plt.plot(dd)

"""
280-370
[[323., 166.],
[182., 235.],
[356.,  79.],
[358.,   1.],

[[326. 209.]
 [182. 231.]
 [174. 149.]
 [325. 193.]
 [325. 127.]
 
 
[193., 296.],
[171.,  58.],
[319.,  68.],
[319.,   9.],

[285., 131.],
       [289., 177.],
       [285., 146.],
       [254., 149.],
       [305., 156.],
       [273.,  82.],
       [294., 117.],
       [315.,  95.],
       [299., 169.],
       [283.,  98.],
       [190., 169.],
       [251., 164.],
"""

