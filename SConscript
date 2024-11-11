from building import *
Import('rtconfig')

src   = []
cwd   = GetCurrentDir()

# add bmp180 src files.
if GetDepend('PKG_BMP180_USING_SENSOR_V1'):
    src += ['bmp180.c']

if GetDepend('PKG_USING_BMP180_SAMPLE'):
    src += Glob('bmp180_sample.c')
    
# add bmp180 include path.
path  = [cwd]

# add src and include to group.
group = DefineGroup('bmp180', src, depend = ['PKG_USING_BMP180'], CPPPATH = path)

Return('group')