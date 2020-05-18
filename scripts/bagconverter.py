import numpy as np
import rosbag
bag=rosbag.Bag('bag.bag')
data=[]
for topic,msg,t in bag.read_messages(topics='/odometry'):
    data.append([t.to_time(),msg.x,msg.y,msg.th])
bag.close()
data=np.array(data)
np.save('data',data)
print("Saved output, dims: ",data.shape)
