import blast
import sys, os, subprocess

my_path = os.path.dirname(os.path.abspath(__file__))

a = []
if "--sim" in sys.argv:
    a = ["--sim",]

subprocess.call(["python", my_path + "/../blast_server/blast_api.py", my_path,] + a + ["--test",])



