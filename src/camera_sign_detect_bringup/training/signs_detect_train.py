#!/usr/bin/env python
# coding: utf-8

# In[11]:


# !pip uninstall -y urllib3 requests requests-toolbelt roboflow

get_ipython().system('pip install "urllib3<2" "requests<3" requests-toolbelt roboflow')


# In[ ]:


from roboflow import Roboflow
rf = Roboflow(api_key="XX")
project = rf.workspace("selfdriving-car-qtywx").project("self-driving-cars-lfjou")
version = project.version(4)
dataset = version.download("yolo26")
                


# In[1]:


from ultralytics import YOLO


# In[16]:


MODEL_BASE='yolo26n.pt'


# In[17]:


model=YOLO(MODEL_BASE)


# In[18]:


data_yaml = f"{dataset.location}/data.yaml"  
model.train(
    data=data_yaml,
    optimizer="AdamW",
    workers=8,
    epochs=50,
    imgsz=640,
    batch=16,
    patience=5,
    project="sign-detect",
    name="yolo_26n",
)


# In[ ]:


# continue for some more epochs

model = YOLO("runs/detect/sign-detect/yolo_26n/weights/last.pt")

model.train(
    data=data_yaml,
    epochs=50,  
    imgsz=640,
    batch=16,
    optimizer="AdamW",
    project="sign-detect",
    name="yolo_26n_finetune",
    lr0=0.001,      # ↓ important change
    lrf=0.01   
)


# In[4]:


best = YOLO('runs/detect/sign-detect/yolo_26n_finetune2/weights/best.pt')


# In[ ]:


result = best.predict(source='Self-Driving-Cars-4/valid/images/000001_jpg.rf.7d5747012e9b548e833a6a45352497ac.jpg',conf=0.4, show= False)


# In[6]:


result[0].show()


# In[ ]:


# export to ONNX
## the parameters are needed to be compatabile with ros_yolo_cpp
best.export(format='onnx',imgsz=640 ,dynamic=False,simplify=True ,opset=12)


# In[2]:


onnx = YOLO('runs/detect/sign-detect/yolo_26n_finetune2/weights/best.onnx')

result_onnx = onnx.predict(source='Self-Driving-Cars-4/valid/images/000001_jpg.rf.7d5747012e9b548e833a6a45352497ac.jpg',conf=0.4, show= False)

result_onnx[0].show()


# In[ ]:




