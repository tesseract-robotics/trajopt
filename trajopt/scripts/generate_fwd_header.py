import argparse
parser = argparse.ArgumentParser()
parser.add_argument("headers", nargs="*")
parser.add_argument("--out")
parser.add_argument("--ns", nargs="*")
args = parser.parse_args()

assert args.out is not None

import os.path as osp
import source_analysis

source_analysis.make_fwd_header(args.headers, args.out, args.ns);
