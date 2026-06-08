
"""
Configuration file for NavOA.

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

device = torch.device("cpu")
time_print = False

def np_to_tensor(array):

    if array is None:
        return None

    # If already a tensor, just move/cast it.
    if torch.is_tensor(array):
        return array.to(device=device, dtype=torch.float32)

    # Scalars
    if np.isscalar(array):
        return torch.tensor(array, dtype=torch.float32, device=device)

    # Robustly handle any array-like (NumPy arrays, matrices, lists, etc.)
    # Use as_tensor instead of from_numpy to avoid strict NumPy type coupling.
    return torch.as_tensor(array, dtype=torch.float32, device=device)

def tensor_to_np(tensor):

    if tensor is None:
        return None

    tensor = tensor.cpu()
    return tensor.detach().numpy()

def value_to_tensor(value, requires_grad=False):
    
    if value is None:
        return None

    return torch.tensor(value, dtype=torch.float32, requires_grad=requires_grad).to(device)

def to_device(tensor):
    return tensor.to(device)