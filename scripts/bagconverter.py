import numpy as np
import rosbag
bag=rosbag.Bag('bag.bag')
data=[]
for topic,msg,t in bag.read_messages(topics='/encoders'):
    data.append([t.to_time(),msg.leftVel])
bag.close()
data=np.array(data)
np.save('data',data)
print("Saved output, dims: ",data.shape)
