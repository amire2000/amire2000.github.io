

sudo systemctl daemon-reload
sudo systemctl restart docker

```
docker run --runtime=nvidia --rm nvidia/cuda:9.0-base nvidia-smi

+-----------------------------------------------------------------------------+
| NVIDIA-SMI 390.116                Driver Version: 390.116                   |
|-------------------------------+----------------------+----------------------+
| GPU  Name        Persistence-M| Bus-Id        Disp.A | Volatile Uncorr. ECC |
| Fan  Temp  Perf  Pwr:Usage/Cap|         Memory-Usage | GPU-Util  Compute M. |
|===============================+======================+======================|
|   0  GeForce GT 630M     Off  | 00000000:01:00.0 N/A |                  N/A |
| N/A   52C    P8    N/A /  N/A |    234MiB /  1985MiB |     N/A      Default |
+-------------------------------+----------------------+----------------------+
                                                                               
+-----------------------------------------------------------------------------+
| Processes:                                                       GPU Memory |
|  GPU       PID   Type   Process name                             Usage      |
|=============================================================================|
|    0                    Not Supported                                       |
+-----------------------------------------------------------------------------+

```


# AirSim
```
python build_airsim_image.py \
   --base_image=nvidia/cudagl:9.0-devel-ubuntu16.04 \
   --target_image=airsim_binary:9.0-devel-ubuntu16.04
```

```
./run_airsim_image_binary.sh airsim_binary:9.0-devel-ubuntu16.04 Blocks/Blocks.sh -windowed -ResX=1080 -ResY=720
```