import os

class ZL_FILE(object):
    # 初始化
    def __init__(self,filename='action.txt'):
        self.filename = filename
    
    # 写文件
    def write(self, content): 
        with open(self.filename, 'a') as f:
            f.write(content)                 # 写文件
            f.flush()                        # 刷新
        
    # 读文件
    def read(self): 
        with open(self.filename, 'r') as f:
            return f.read()                  # 读文件

    # 清空文件
    def clear(self):
        os.remove(self.filename)
        f = open(self.filename,'w')
        f.close()
      
    # 读取某一行
    def readline(self,line):
        with open(self.filename, 'r') as f:
            for i in range(line):
                f.readline()  
            return f.readline()              # 读文件
        
if __name__=='__main__':
    a = ZL_FILE('action.txt')                # 实例化
    #a.clear()

    '''
    for i in range(10):
        a.write('{#%03dP1209T0000!}\n' % i)  # 写

    data = a.readline(4)                     # 读
    print(data)
    '''