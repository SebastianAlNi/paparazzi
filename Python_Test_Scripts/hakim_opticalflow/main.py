

import numpy as np
import extract_information_flow_field as OF


 #if not os.path.exists('pics'): 
        
# Change the image numbers below to answer the questions:
 
for x in range(2,400):
    img_nr_1 = x;
    img_nr_2 = x+1;
    points_old, points_new, flow_vectors,c, d = OF.show_flow(x,img_nr_1, img_nr_2);
    length=len(points_old)
  #  print (c)
  #  print (points_old)
  #  print(length)


