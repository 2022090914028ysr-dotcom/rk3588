"""
DUNE (Deep Unfolded Neural Encoder) is the core class of the PAN class. It maps the point flow to the latent distance space: mu and lambda. 

Developed by Ruihua Han
Copyright (c) 2025 Ruihua Han <hanrh@connect.hku.hk>

NavOA planner is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

NavOA planner is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with NavOA planner. If not, see <https://www.gnu.org/licenses/>.
"""


import torch
import numpy as np
from math import inf
from . import ObsPointNet, DUNETrain
from ..configuration import np_to_tensor, to_device
from ..util import time_it, file_check, repeat_mk_dirs
from typing import Optional
import sys

# Clear any cached imports
if 'rknnlite' in sys.modules:
    del sys.modules['rknnlite']
if 'rknnlite.api' in sys.modules:
    del sys.modules['rknnlite.api']

try:
    # Try RKNNLite first (for newer versions)
    from rknnlite.api import RKNNLite
    RKNN = RKNNLite
    print(f"[DUNE] Using RKNNLite API, constants available: {hasattr(RKNN, 'NPU_CORE_AUTO')}")
except ImportError as e:
    print(f"[DUNE] RKNNLite import failed: {e}")
    try:
        # Fall back to old RKNN API
        from rknn.api import RKNN
        print("[DUNE] Using legacy RKNN API")
    except ImportError:
        RKNN = None
        print("[DUNE] RKNN not available")
        
        
class DUNE(torch.nn.Module):

    def __init__(
        self,
        receding: int = 10,
        checkpoint=None,
        robot=None,
        dune_max_num: int = 100,
        train_kwargs: dict = dict(),
        use_rknn: bool = False,
        rknn_path: Optional[str] = None,
        rknn_target: str = "",
        rknn_core: str = "auto",
    ) -> None:
        super(DUNE, self).__init__()
  
        self.T = receding
        self.max_num = dune_max_num

        self.robot = robot

        self.G = np_to_tensor(robot.G)
        self.h = np_to_tensor(robot.h)
        self.edge_dim = self.G.shape[0]
        self.state_dim = self.G.shape[1]

        self.use_rknn = use_rknn
        self.rknn_path = rknn_path if rknn_path is not None else checkpoint
        self.rknn = None
        self._rknn_ready = False
        self._dbg_counter = 0  # light-weight debug ticker
        self.rknn_target = rknn_target
        self.rknn_core = rknn_core

        if self.use_rknn:
            self.model = None
            self._init_rknn_runtime()
        else:
            self.model = to_device(ObsPointNet(2, self.edge_dim))
            self.load_model(checkpoint, train_kwargs)

        self.obstacle_points = None
        self.min_distance = inf


        
    @time_it('- dune forward')
    def forward(self, point_flow: list[torch.Tensor], R_list: list[torch.Tensor], obs_points_list: list[torch.Tensor]=[]) -> tuple[list[torch.Tensor], list[torch.Tensor], list[torch.Tensor]]:

        '''
        map point flow to the latent distance features: lam, mu

        Args:
            point_flow: point flow under the robot coordinate, list of (state_dim, num_points); list length: T+1
            R_list: list of Rotation matrix, list of (2, 2), used to generate the lam from mu; list length: T
            obstacle_points: tensor of shape (2, num_points), global coordinate; 

        Returns: 
            lam_list: list of lam tensor, each element is a tensor of shape (state_dim, num_points); list length: T+1
            mu_list: list of mu tensor, each element is a tensor of shape (edge_number, num_points); list length: T+1
            sort_point_list: list of point tensor, each element is a tensor of shape (state_dim, num_points); list length: T+1; 
        '''

        mu_list, lam_list, sort_point_list = [], [], []
        self.obstacle_points = obs_points_list[0] # current obstacle points considered in the dune at time 0

        total_points = torch.hstack(point_flow)

        # light debug: every 50 calls, report whether RKNN is used and tensor sizes
        self._dbg_counter += 1
        if self._dbg_counter % 50 == 1:
            path = self.rknn_path if self.use_rknn else getattr(self, 'abs_checkpoint_path', None)
            print(f"[DUNE][dbg] use_rknn={self.use_rknn} ready={self._rknn_ready} path={path} total_points={total_points.shape}")
        
        # map the point flow to the latent distance features mu
        if self.use_rknn:
            total_mu = self._rknn_infer(total_points)
        else:
            with torch.no_grad():
                total_mu = self.model(total_points.T).T
        
        for index in range(self.T+1):
            num_points = point_flow[index].shape[1]
            mu = total_mu[:, index*num_points : (index+1)*num_points]
            R = R_list[index]
            p0 = point_flow[index]
            lam = (- R @ self.G.T @ mu)

            if mu.ndim == 1:
                mu = mu.unsqueeze(1)
                lam = lam.unsqueeze(1)

            distance = self.cal_objective_distance(mu, p0)

            if index == 0: 
                self.min_distance = torch.min(distance) 
            
            sort_indices = torch.argsort(distance)

            mu_list.append(mu[:, sort_indices])
            lam_list.append(lam[:, sort_indices])
            sort_point_list.append(obs_points_list[index][:, sort_indices])

        return mu_list, lam_list, sort_point_list


    def cal_objective_distance(self, mu: torch.Tensor, p0: torch.Tensor) -> torch.Tensor:

        '''
        input: 
            mu: (edge_dim, num_points)
            p0: (state_dim, num_points)   
        output:
            distance:  mu.T (G @ p0 - h),  (num_points,)
        ''' 

        temp = (self.G @ p0 - self.h).T.unsqueeze(2)
        muT = mu.T.unsqueeze(1)

        distance = torch.squeeze(torch.bmm(muT, temp)) 

        if distance.ndim == 0:
            distance = distance.unsqueeze(0)

        return distance
    


    def load_model(self, checkpoint: Optional[str]=None, train_kwargs: Optional[dict]=None):

        '''
        checkpoint: pth file path of the model
        '''

        try:
            if checkpoint is None:
                raise FileNotFoundError

            self.abs_checkpoint_path = file_check(checkpoint)
            self.model.load_state_dict(torch.load(self.abs_checkpoint_path, map_location=torch.device('cpu')))
            to_device(self.model)
            self.model.eval()

        except FileNotFoundError:

            if train_kwargs is None or len(train_kwargs) == 0:
                print('No train kwargs provided. Default value will be used.')
                train_kwargs = dict()
            
            direct_train = train_kwargs.get('direct_train', False)

            if direct_train:
                print('train or test the model directly.')
                return 

            if self.ask_to_train():
                self.train_dune(train_kwargs)

                if self.ask_to_continue():
                    self.model.load_state_dict(torch.load(self.full_model_name, map_location=torch.device('cpu')))
                    to_device(self.model)
                    self.model.eval()
                else:
                    print('You can set the new model path to the DUNE class to use the trained model.') 

            else:
                print('Can not find checkpoint. Please check the path or train first.')
                raise FileNotFoundError


    def _init_rknn_runtime(self):
        if not self.use_rknn:
            return

        if RKNN is None:
            raise ImportError('rknn-toolkit2 is not installed; cannot run RKNN inference')

        if self.rknn_path is None:
            raise FileNotFoundError('No RKNN model path provided')

        # allow absolute path to bypass pkg-relative resolution
        from pathlib import Path
        rp = Path(self.rknn_path)
        if rp.is_absolute():
            if not rp.exists():
                raise FileNotFoundError(f"RKNN path does not exist: {rp}")
            self.abs_checkpoint_path = str(rp)
        else:
            self.abs_checkpoint_path = file_check(self.rknn_path)

        print(f"[DUNE][RKNN] loading model: {self.abs_checkpoint_path}")
        self.rknn = RKNN()
        ret = self.rknn.load_rknn(self.abs_checkpoint_path)
        if ret != 0:
            raise RuntimeError(f'load_rknn failed with code {ret}')

        # map core string to RKNN mask
        core_map = {}
        try:
            core_map = {
                'auto': RKNN.NPU_CORE_AUTO,
                '0': RKNN.NPU_CORE_0,
                '1': RKNN.NPU_CORE_1,
                '2': RKNN.NPU_CORE_2,
                '0_1': RKNN.NPU_CORE_0_1,
                '0_1_2': RKNN.NPU_CORE_0_1_2,
                'all': RKNN.NPU_CORE_ALL,
            }
        except Exception:
            pass

        core_mask = core_map.get(str(self.rknn_core).lower(), 0)
        
        # SIMPLE FIX: Convert string "None" to Python None, and handle case
        if self.rknn_target in ["None", "null", ""]:
            actual_target = None
        elif self.rknn_target == "":
            actual_target = ""  # Try uppercase
        else:
            actual_target = self.rknn_target
            
        print(f"[DUNE][RKNN] init_runtime target={actual_target} core={self.rknn_core} mask={hex(core_mask) if isinstance(core_mask,int) else core_mask}")
        
        try:
            if core_mask:
                ret = self.rknn.init_runtime(target=actual_target, core_mask=core_mask)
            else:
                ret = self.rknn.init_runtime(target=actual_target)
        except TypeError:
            # older toolkit without core_mask param
            ret = self.rknn.init_runtime(target=actual_target)
        except Exception as e:
            # If that fails, try without target
            print(f"[DUNE][RKNN] First attempt failed: {e}, trying without target...")
            if core_mask:
                ret = self.rknn.init_runtime(core_mask=core_mask)
            else:
                ret = self.rknn.init_runtime()
                
        if ret != 0:
            raise RuntimeError(f'init_runtime failed with code {ret}')

        self._rknn_ready = True
        try:
            # not all toolkits expose get_sdk_version; ignore failures
            ver = self.rknn.get_sdk_version()
            print(f"[DUNE][RKNN] runtime ready, sdk_version={ver}")
        except Exception:
            print("[DUNE][RKNN] runtime ready")

    def _rknn_infer(self, total_points: torch.Tensor) -> torch.Tensor:
        """Infer mu using RKNN runtime.

        Args:
            total_points: (2, total_num)
        Returns:
            torch.Tensor of shape (edge_dim, total_num)
        """
        if self.rknn is None or not self._rknn_ready:
            raise RuntimeError('RKNN runtime is not initialized')

        pts_all = total_points.T.detach().cpu().numpy().astype(np.float32)  # (N,2)

        # RKNN model was exported with fixed point count (e.g., 100). Chunk and pad to fixed size.
        chunk_size = 100
        mu_list = []
        for start in range(0, pts_all.shape[0], chunk_size):
            end = min(start + chunk_size, pts_all.shape[0])
            this_len = end - start
            buf = np.zeros((1, chunk_size, 2), dtype=np.float32)
            buf[0, :this_len, :] = pts_all[start:end, :]

            outputs = self.rknn.inference(inputs=[buf])
            if outputs is None or len(outputs) == 0:
                raise RuntimeError('RKNN inference returned no outputs')

            mu_np = outputs[0]
            if mu_np.ndim == 3 and mu_np.shape[0] == 1:
                mu_np = mu_np[0]

            if mu_np.ndim != 2:
                raise RuntimeError(f'Unexpected RKNN output shape: {mu_np.shape}')

            mu_list.append(mu_np[:this_len, :])

            if self._dbg_counter % 50 == 1:
                print(f"[DUNE][dbg] rknn chunk start={start} len={this_len} input={buf.shape} output={mu_np.shape}")

        mu_concat = np.concatenate(mu_list, axis=0).astype(np.float32, copy=False)  # (N, edge_dim)
        # torch.from_numpy is fastest but can fail in some mixed/embedded numpy setups.
        try:
            mu_tensor = torch.from_numpy(mu_concat.T)  # (edge_dim, N)
        except TypeError:
            # Fallback: avoid the NumPy C-API path by materializing a Python list.
            mu_tensor = torch.tensor(mu_concat.T.tolist(), dtype=torch.float32)
        return mu_tensor


    def train_dune(self, train_kwargs):

        model_name = train_kwargs.get("model_name", self.robot.name)

        checkpoint_path = sys.path[0] + '/model' + '/' + model_name
        checkpoint_path = repeat_mk_dirs(checkpoint_path)
        
        self.train_model = DUNETrain(self.model, self.G, self.h, checkpoint_path)
        self.full_model_name = self.train_model.start(**train_kwargs)
        print('Complete Training. The model is saved in ' + self.full_model_name)

    def ask_to_train(self):
        
        while True:
            choice = input("Do not find the DUNE model; Do you want to train the model now, input Y or N:").upper()
            if choice == 'Y':
                return True
            elif choice == 'N':
                print('Please set the your model path for the DUNE layer.')
                sys.exit()
            else:
                print("Wrong input, Please input Y or N.")


    def ask_to_continue(self):
        
        while True:
            choice = input("Do you want to continue the case running, input Y or N:").upper()
            if choice == 'Y':
                return True
            elif choice == 'N':
                print('exit the case running.')
                sys.exit()
            else:
                print("Wrong input, Please input Y or N.")


    @property
    def points(self):
        '''
        point considered in the dune layer
        '''

        return self.obstacle_points



