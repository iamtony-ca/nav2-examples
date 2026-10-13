import time, rclpy
from rclpy.qos import qos_profile_sensor_data as Q
from sensor_msgs.msg import PointCloud2, Image
from tf2_msgs.msg import TFMessage
from zed_msgs.msg import Heartbeat, ObjectsStamped
p='/zed_multi/'
T={'F':(p+'zed_front/point_cloud/cloud_registered',PointCloud2),'R':(p+'zed_rear/point_cloud/cloud_registered',PointCloud2),
   'G':(p+'zed_right/left/gray/rect/image',Image),'OD':(p+'zed_front/obj_det/objects',ObjectsStamped),
   'hF':(p+'zed_front/status/heartbeat',Heartbeat),'hR':(p+'zed_rear/status/heartbeat',Heartbeat),
   'hG':(p+'zed_right/status/heartbeat',Heartbeat),'tf':('/tf',TFMessage)}
rclpy.init(); n=rclpy.create_node('diag_gap')
t0=time.time(); last={k:t0 for k in T}; c={k:0 for k in T}; g={k:0. for k in T}; lag={k:0. for k in T}
def cb(k):
    def f(m):
        t=time.time(); g[k]=max(g[k],t-last[k]); last[k]=t; c[k]+=1
        h=getattr(m,'header',None)
        if h: lag[k]=t-h.stamp.sec-h.stamp.nanosec*1e-9
    return f
for k,(tp,ty) in T.items(): n.create_subscription(ty,tp,cb(k),Q)
def tick():
    t=time.time(); print('%.2f '%t+' '.join('%s:%d/%d/%.2f'%(k,c[k],1000*max(g[k],t-last[k]),lag[k]) for k in T),flush=True)
    for k in T: c[k]=0; g[k]=0.
n.create_timer(1.0,tick); rclpy.spin(n)
