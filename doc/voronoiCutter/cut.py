import os

#获取当前路径
d = os.path.dirname(__file__)
e = os.getcwd()
print("当前路径为：",d)
print(e)

#a=os.walk(os.path.dirname(d)) #会返回容器类型子目录，很麻烦
a=os.listdir(d)
print("返回类型为:",type(a))  #返回了本人钟爱的list
print(a)
b=[]

#整理列表 收集图片
for i in a :
    if i[-3:] in ['png']:
         b.append(i)
print(b)


from PIL import Image
for path in b:
    img = Image.open(path)
    print(img.size)
    cropped = img.crop((5, 5, img.size[0]-5, img.size[1]-5))  # (left, upper, right, lower)
    cropped.save(path)