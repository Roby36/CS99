
import shutil

# Specify the source and destination file paths
source_file = 'params0.json' # private to this file 
destination_file = 'params.json'

def main():
    # Copy the content from source to destination
    shutil.copyfile(source_file, destination_file)

if __name__ == '__main__':
    main()