import os


def create_new_directory(base_path):
    if not os.path.exists(base_path):
        raise ValueError("Base path does not exist.")

    # List all entries in the base path
    existing_entries = os.listdir(base_path)

    # Filter out entries that are not directories or not numerical
    numerical_dirs = [d for d in existing_entries if d.isdigit() and os.path.isdir(os.path.join(base_path, d))]

    # Find the next numerical value
    next_number = 1
    if numerical_dirs:
        # Find the maximum number and add 1
        max_number = max(map(int, numerical_dirs))
        next_number = max_number + 1

    # Create the new directory
    new_dir_path = os.path.join(base_path, str(next_number).zfill(3))
    os.makedirs(new_dir_path)
    return new_dir_path