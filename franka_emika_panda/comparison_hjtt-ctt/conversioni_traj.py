import numpy as np
import pandas as pd
import scipy.io

def read_file_mod(path: str, sheet_name: str = None):
    """
    Read a trajectory Excel file containing columns:
    x, y, z, time.
    
    Returns:
        N×3 numpy array [x, y, z].
    """

    # Load file
    df = pd.read_excel(path, sheet_name=sheet_name)

    # Columns required
    required_cols = ['x', 'y', 'z']

    for col in required_cols:
        if col not in df.columns:
            raise ValueError(f"[ERRORE] La colonna '{col}' non è presente in {path}")

    # Extract xyz only
    traj = df[['x', 'y', 'z']].to_numpy()

    return traj


def save_ctt_hjtt_to_mat(ctt_file, hjtt_file, mat_filename, freq=60.0):
    """
    Load CTT and HJTT trajectories from Excel,
    align lengths, build time vector, and save a combined .mat file.
    """

    # ----- Read files -----
    ctt = read_file_mod(ctt_file, 'IK')
    hjtt = read_file_mod(hjtt_file, 'IK')

    # ----- Align lengths -----
    min_len = min(len(ctt), len(hjtt))
    ctt = ctt[-min_len:, :]
    hjtt = hjtt[-min_len:, :]

    # ----- Time vector -----
    time = np.arange(min_len) / freq

    # ----- Build data dictionary -----
    data = {
        'time': time,
        'x_ctt':  ctt[:, 0],
        'y_ctt':  ctt[:, 1],
        'z_ctt':  ctt[:, 2],
        'x_hjtt': hjtt[:, 0],
        'y_hjtt': hjtt[:, 1],
        'z_hjtt': hjtt[:, 2],
    }

    # ----- Save to .mat -----
    scipy.io.savemat(mat_filename, data)
    print(f"[INFO] File .mat creato → {mat_filename}")


# ============================================================
# USO
# ============================================================
smf = False
if smf == True: 
    ctt_file  = 'sim-coppelia-finali/risultati/smf/traj_smf_ctt.xlsx'
    hjtt_file = 'sim-coppelia-finali/risultati/smf/traj_smf_hjtt.xlsx'
    output_mat = 'sim-coppelia-finali/risultati/smf/traj_ctt_vs_hjtt.mat'
else: 
    ctt_file  = 'sim-coppelia-finali/risultati/efm/traj_efm_ctt.xlsx'
    hjtt_file = 'sim-coppelia-finali/risultati/efm/traj_efm_hjtt.xlsx'
    output_mat = 'sim-coppelia-finali/risultati/efm/traj_ctt_vs_hjtt.mat'
save_ctt_hjtt_to_mat(ctt_file, hjtt_file, output_mat)
