import pandas as pd

def split_csv(file_path, num_parts=5):
    # Load the CSV file
    df = pd.read_csv(file_path)
    
    # Calculate the number of rows to put in each file
    rows_per_part = len(df) // num_parts
    remainder = len(df) % num_parts
    
    # Split and save parts
    start = 0
    for part in range(num_parts):
        # Calculate end row for this part
        end = start + rows_per_part + (1 if part < remainder else 0)
        # Slice the DataFrame for this part
        df_part = df.iloc[start:end]
        # Save this part to a new CSV file
        df_part.to_csv(f'{file_path.rsplit(".", 1)[0]}_part{part+1}.csv', index=False)
        # Update start row for next part
        start = end

# Example usage
split_csv('/home/nic/dev/ATTACKS-SW/hitl_dataset_13.csv')
