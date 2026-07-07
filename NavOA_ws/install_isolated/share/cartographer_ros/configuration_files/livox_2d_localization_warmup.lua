-- =============================================================================
-- livox_2d_localization_warmup.lua
-- Ԥ��ģʽ��ʹ�ò�ͬ�� TF frame �����������������еľɽڵ��ͻ��
-- ����  map_warmup �� odom_warmup���� tf_frame_switch �ڵ����л���ת����
-- =============================================================================
include "livox_2d_localization.lua"

-- ���� frame ��ΪԤ��ר����
options.map_frame = "map_warmup"
options.published_frame = "odom_warmup"
options.odom_frame = "odom_warmup"
