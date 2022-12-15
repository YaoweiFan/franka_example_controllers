# 读取 real 文件
class record:
    def __init__(self):
        self.r = []


recordList = []
curr_record = record()
num_record = 0

realPx = []
realPy = []
realPz = []
realQx = []
realQy = []
realQz = []

realPeef_quatx = []
realPeef_quaty = []
realPeef_quatz = []
realPeef_quatw = []
realQeef_quatx = []
realQeef_quaty = []
realQeef_quatz = []
realQeef_quatw = []

realPpegx = []
realPpegy = []
realPpegz = []
realQpegx = []
realQpegy = []
realQpegz = []

realPpthx = []
realPpthy = []
realPpthz = []
realQpthx = []
realQpthy = []
realQpthz = []

realPacts = []
realQacts = []

realPpubx = []
realPpuby = []
realPpubz = []
realQpubx = []
realQpuby = []
realQpubz = []

myDict = {'Px':realPx, 'Py':realPy, 'Pz':realPz,
          'Peef_quatx':realPeef_quatx, 'Peef_quaty':realPeef_quaty, 'Peef_quatz':realPeef_quatz, 'Peef_quatw':realPeef_quatw, 
          'Ppegx':realPpegx, 'Ppegy':realPpegy, 'Ppegz':realPpegz, 
          'Ppthx':realPpthx, 'Ppthy':realPpthy, 'Ppthz':realPpthz, 
          'Pacts':realPacts,
          'Ppubx':realPpubx, 'Ppuby':realPpuby, 'Ppubz':realPpubz,
          'Qx':realQx, 'Qy':realQy, 'Qz':realQz,
          'Qeef_quatx':realQeef_quatx, 'Qeef_quaty':realQeef_quaty, 'Qeef_quatz':realQeef_quatz, 'Qeef_quatw':realQeef_quatw, 
          'Qpegx':realQpegx, 'Qpegy':realQpegy, 'Qpegz':realQpegz, 
          'Qpthx':realQpthx, 'Qpthy':realQpthy, 'Qpthz':realQpthz, 
          'Qacts':realQacts,
          'Qpubx':realQpubx, 'Qpuby':realQpuby, 'Qpubz':realQpubz
}

for line in open("/home/fyw/Documents/projects/panda_ros/franka_panda_control_ws/src/franka_ros/franka_example_controllers/scripts/data/real"):
    if line.find('Enter') != -1:
        # 保存老的记录，创建新的记录
        if num_record != 0:
            recordList.append(curr_record)
        curr_record = record()
        num_record += 1

    if line.find('|') != -1:
        splitList = line.split('|')
        cr = []
        for item in splitList:
            if item != '' and item != '\n': 
                cr.append(item.strip())
        curr_record.r.append(cr)            

for item in recordList:
    index = 0
    while index < len(item.r):
        for i in range(len(item.r[index])):
            myDict[item.r[index][i]].append(float(item.r[index+1][i]))
        index += 2
