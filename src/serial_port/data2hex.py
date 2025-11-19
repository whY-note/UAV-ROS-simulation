import struct
import pandas as pd
import numpy as np
FRAME_HEADER=b'\xAA\xBB'
FRAME_FOOTER=b'\xCC\xDD'

def send_floats( float_list):
    # 发送数据
    
    if len(float_list) != 12:
        print("必须提供12个float数据")
        return None
    
    try:
        # 打包数据（4字节 × 12 = 48字节）
        data = struct.pack('<12f', *float_list)
        
        # 添加帧头帧尾 (2+48+2=52字节)
        data=FRAME_HEADER+data+FRAME_FOOTER
        
        # 发送
        return data
        
    except struct.error as e:
        print(f"打包失败: {e}")
        return None
        
        
df=pd.read_excel("xk_history.xlsx",index_col=0)
print(df)
print(df.shape[0])
output=[]

for i in range(df.shape[0]):
    print(i)
    float_array=np.array(df.loc[i,:])
    print(f"original float array:{float_array}")
    send_data=send_floats(float_array).hex()
    print(f"send data(hex): {send_data}")
    output.append({"float_array":float_array,"send data hex":send_data})
    
df_output=pd.DataFrame(output)
df_output.to_excel("send_data.xlsx",index=False)