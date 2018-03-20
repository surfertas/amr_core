# @author Tasuku Miura
# @brief Utils used for PyTorch training


def save_checkpoint(state, is_best, file_name='/output/checkpoint.pth.tar'):
    """Save checkpoint if a new best is achieved"""
    if is_best:
        print ("=> Saving a new best")
        torch.save(state, file_name)  # save checkpoint
    else:
        print ("=> Validation Accuracy did not improve")


def create_dir(dir_name):
    if not os.path.isdir(dir_name):
        os.makedirs(dir_name)
