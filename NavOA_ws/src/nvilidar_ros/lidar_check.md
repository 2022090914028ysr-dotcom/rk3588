python3 -c "
> import rospy,math
> from sensor_msgs.msg import LaserScan
> rospy.init_node('chk')
> m=rospy.wait_for_message('/scan',LaserScan)
> r=m.ranges
> f=[x for x in r if math.isfinite(x)]
> print('points',len(r))
> print('zeros',sum(x==0 for x in r))
> print('inf',sum(math.isinf(x) for x in r))
> print('nan',sum(math.isnan(x) for x in r))
> print('above range_max',sum(x>m.range_max for x in f))
> print('valid',sum(m.range_min<=x<=m.range_max for x in f))
> print('largest valid',max([x for x in f if x<=m.range_max] or [0]))
> print('range_min',m.range_min,'range_max',m.range_max)
> print('angle span deg',math.degrees(m.angle_max-m.angle_min))
> print('time_increment',m.time_increment)
> "



python3 -c "
> import rospy,math
> from sensor_msgs.msg import LaserScan
> rospy.init_node('chk')
> m=rospy.wait_for_message('/scan',LaserScan)
> r=list(m.ranges);n=len(r);d=math.degrees
> runs=[];s=None
> for i,x in enumerate(r):
>     if x==0 and s is None: s=i
>     if x!=0 and s is not None: runs.append([s,i-1]); s=None
> if s is not None: runs.append([s,n-1])
> if len(runs)>1 and runs[0][0]==0 and runs[-1][1]==n-1:
>     runs[0][0]=runs[-1][0]-n; runs.pop()
> ang=lambda i: d(m.angle_min+(i%n)*m.angle_increment)
> big=[(a,b) for a,b in runs if b-a+1>=5]
> for a,b in big:
>     print('zeros from %.1f to %.1f deg, width %.1f deg'%(ang(a),ang(b),(b-a+1)*d(m.angle_increment)))
> print('zeros in blocks',sum(b-a+1 for a,b in big),'isolated zeros',sum(b-a+1 for a,b in runs if b-a+1<5),'total',sum(x==0 for x in r))
> "


tigervncserver :2 -geometry 1280x800 -depth 24 -localhost no

