# RKNN Ǩ�������Ķ�˵��

## Ŀ��
- �� DUNE ��ģ�ʹ� PyTorch CPU ����Ǩ�Ƶ� RK3588 NPU��rknn-toolkit2����
- ���ֻ�����������δ���� RKNN ������ʧ�ܣ��ɼ���ʹ��ԭʼ .pth ģ���� CPU ��������

## �ؼ��Ķ�һ��

- �������������� rknn ѡ������Ŀ��ƣ�
  - �ļ���`src/NavOA_ros/example/gazebo_limo/launch/NavOA_gazebo_limo.launch`
  - ���� arg/param��`use_rknn`��Ĭ�� true����`dune_rknn`��`rknn_target`��Ĭ�� rk3588����`rknn_core`��Ĭ�� auto�������ݸ��ڵ㡣

- �ڵ�㣨��ȡ rknn ���ò�͸����
  - �ļ���`src/NavOA_ros/src/NavOA_core.py`
  - ������ȡ `~use_rknn`/`~dune_rknn`/`~rknn_target`/`~rknn_core`����������ʱ��־��ӡ������Щ����͸���� PAN/DUNE��

- PAN �㣨�����·���
  - �ļ���`src/NavOA_ros/NavOA/NavOA/blocks/pan.py`
  - PAN ���캯������ `rknn_target`��`rknn_core`�����´��� DUNE��

- DUNE �㣨RKNN ������ֿ飩
  - �ļ���`src/NavOA_ros/NavOA/NavOA/blocks/dune.py`
  - ���� RKNN ����ʱ��ʼ����`init_runtime(target=rk3588, core_mask=...)`��֧�� core ѡ��auto/0/1/2/0_1/0_1_2/all����
  - ����·��֧�֣�`dune_rknn` Ϊ����·��ʱ���ٴ���ƴ�ӡ�
  - �����ֿ�����䣺RKNN ģ�͹̶�ÿ�� 100 �㣬DUNE �Ὣ���ư� 100 �ֿ顢������������ٰ�ʵ�ʳ���ƴ�ӻ� `mu`������ά�Ȳ�ƥ�䡣
  - ������־������/��ʼ��/ÿ 50 ��ǰ���ӡ use_rknn��ready ״̬����������ߴ磬����ȷ���Ƿ��ߵ� NPU��

## CPU �汾 vs NPU �汾����

- ģ�ͼ���
  - CPU������ `.pth` �� PyTorch `ObsPointNet`���� CPU/torch ���С�
  - NPU������ `.rknn`��ͨ�� rknn-toolkit2 �� NPU �������� `use_rknn=false` ���ʼ��ʧ�ܣ��Կ�ʹ�� CPU ·����

- ����ߴ�
  - CPU��ԭ��֧�����������������
  - NPU��RKNN ģ�͹̶�ÿ�� 100 �㣻DUNE �����Զ��ֿ顢��䡢�ü���ȷ�����������������롣

- ����ʱѡ��
  - ͨ�� launch ���� `use_rknn` ���ƣ�`rknn_target` ǿ��Ŀ��Ϊ rk3588��`rknn_core` ��ָ�������루auto/0/1/2/0_1/0_1_2/all����

- ��־����֤
  - CPU������ ROS ��־��
  - NPU�������ӡ `[NavOA] use_rknn=... target=... core=...`��`[DUNE][RKNN] loading model ...`��`[DUNE][RKNN] init_runtime target=... core=...`���Լ�ÿ 50 �� `[DUNE][dbg] ...`��
  - NPU ���ؿ��� `watch -n 0.2 sudo cat /sys/kernel/debug/rknpu/load` �۲죨�������� debugfs����

## ʹ�÷�����RKNN ģʽ��

```bash
# ȷ�� rknn-toolkit2 ��װ�ڵ�ǰ������������ /dev/rknpu*
roslaunch NavOA_ros NavOA_gazebo_limo.launch \
  use_rknn:=true \
  dune_rknn:=/home/ubuntu/NavOA_ws2/src/NavOA_ros/example/gazebo_limo/pretrain_limo/model_5000_quant.rknn \
  rknn_target:=rk3588 \
  rknn_core:=auto
```

- ����ǿ�Ƶ��ˣ������1��`rknn_core:=1`��
- ������� CPU��`use_rknn:=false`�������� `dune_checkpoint` ָ�� `.pth`��

## ����ļ��б�
- `src/NavOA_ros/example/gazebo_limo/launch/NavOA_gazebo_limo.launch`
- `src/NavOA_ros/src/NavOA_core.py`
- `src/NavOA_ros/NavOA/NavOA/blocks/pan.py`
- `src/NavOA_ros/NavOA/NavOA/blocks/dune.py`

## ��֪ע������
- ȷ��ɨ������Ч�ϰ��㣻���㱻���ˣ�DUNE/RKNN ���ᱻ���ã�NPU ���ؿ���Ϊ 0��
- ������Ȩ�ޣ������ `/dev/rknpu*`��`/sys/kernel/debug/rknpu/load` ����� debugfs��
- rknn-core ��������ԣ��ɰ� toolkit ����֧�� `core_mask`�������Զ�����ֻ�� target��
