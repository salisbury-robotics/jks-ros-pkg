import blast
import sys, os, subprocess

my_path = os.path.dirname(os.path.abspath(__file__))

subprocess.call(["python", my_path + "/../blast_server/blast_api.py", my_path, "--test"])



