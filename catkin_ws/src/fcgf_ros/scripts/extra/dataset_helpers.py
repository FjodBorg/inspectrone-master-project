import os

class Config():
    def __init__(self):
        # every attribute is defined externally
        pass


class IOS(Config):
    def __init__(self, config):
        self.config = config

    def get_choice(self, extension=".npz"):
        frs = ["f", "r", "s"]
        choice = frs[0]

        if any(File.endswith(extension) for File in os.listdir(self.config.dataset_dir)):
            print("files with extension {} exists".format(extension))
            while True:
                # ask user for what to do
                print("Do you want to:")
                print("    {}: Fill out missing [Default]".format(frs[0]))
                print("    {}: Replace files".format(frs[1]))
                print("    {}: Skip to next step".format(frs[2]))
                choice = input().lower().replace(" ", "")
                if choice == "" or choice == frs[0]:
                    choice = frs[0]
                    break
                elif choice == frs[1] or choice == frs[2]:
                    break
            print("[{}] was chosen\n".format(choice))
        else:
            print(
                "No {} files found in {} \nProceeding to {} generation".format(
                    extension, config.dataset_dir, extension
                )
            )
        return choice, frs

class Generator_PCD(Config):
    def __init__(self, config):
        self.config = config
        # self.ios = ios

    