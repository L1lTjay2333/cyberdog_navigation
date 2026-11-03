"""Python 3.6 depth preview encoder. No ROS imports, TF or motion calls."""
import base64,math,struct,time

def depth_preview(image,now):
 if not 1<=image.width<=1280 or not 1<=image.height<=1024:raise ValueError('Invalid image dimensions')
 if image.encoding not in ('16UC1','32FC1'):raise ValueError('Metric depth required')
 unit=2 if image.encoding=='16UC1' else 4
 if not image.width*unit<=image.step<=image.width*unit+4096 or len(image.data)!=image.step*image.height:raise ValueError('Invalid buffer layout')
 stamp=image.header.stamp.sec+image.header.stamp.nanosec/1e9
 if not math.isfinite(stamp) or not -.1<=now-stamp<=1:raise ValueError('Depth timestamp stale or clocks differ')
 stride=max(1,int(math.ceil(max(image.width/160.,image.height/120.))))
 reader=struct.Struct(('>' if image.is_bigendian else '<')+('H' if unit==2 else 'f'))
 values=[];valid=[]
 for y in range(0,image.height,stride):
  for x in range(0,image.width,stride):
   z=reader.unpack_from(image.data,y*image.step+x*unit)[0]*(.001 if unit==2 else 1.)
   mm=int(round(z*1000)) if math.isfinite(z) and .15<=z<=4. else 0
   values.append(mm)
   if mm:valid.append(mm)
 return dict(width=(image.width+stride-1)//stride,height=(image.height+stride-1)//stride,
  data_mm_b64=base64.b64encode(struct.pack('<'+'H'*len(values),*values)).decode('ascii'),
  stamp=stamp,received_unix=now,source_frame=image.header.frame_id,valid_fraction=len(valid)/float(len(values)),
  min_m=min(valid)/1000. if valid else None,max_m=max(valid)/1000. if valid else None,
  display_range_m=[.15,4.],encoding='uint16_le_mm',source_resolution=[image.width,image.height])
