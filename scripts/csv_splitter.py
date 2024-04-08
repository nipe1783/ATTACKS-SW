import pandas as pd

def split_csv(file_path, num_parts=5):
    df = pd.read_csv(file_path)
    rows_per_part = len(df) // num_parts
    remainder = len(df) % num_parts
    start = 0
    for part in range(num_parts):
        end = start + rows_per_part + (1 if part < remainder else 0)
        df_part = df.iloc[start:end]
        df_part.to_csv(f'{file_path.rsplit(".", 1)[0]}_part{part+1}.csv', index=False)
        start = end

split_csv('/home/nic/dev/ATTACKS-SW/hitl_dataset_13.csv')
