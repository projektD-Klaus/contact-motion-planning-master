#!/usr/bin/env python
import numpy as np
import re
import argparse

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Sample wrl files from templates which contain <mean, std, min, max> tags.', formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('input_file', default='wall-rbo-boxes.wrl.template', type=str, help='The template wrl file to read from.')
    parser.add_argument('output_basename', type=str, default='test_', help='the base name of the output (can be a folder plus first part of the file name)')
    parser.add_argument('--samples', type=int, default = 1, help='Number of samples to draw.')
    parser.add_argument('--ignoreminmax', default=False, action='store_true')
    args = parser.parse_args()
    
    floating_number = '[+-]?([0-9]*[.])?[0-9]+'
    four_numbers = re.compile('<'+floating_number+','+floating_number+','+floating_number+','+floating_number+'>')

    output_name = (args.output_basename + "{:0%id}.wrl") % (len(str(args.samples)))

    with open(args.input_file, 'r') as f:
        original = f.read()
    
    for i in range(args.samples):
        txt = original
        for match in four_numbers.finditer(original):
            l = original[match.start()+1:match.end()-1].split(',')
            l = [float(x) for x in l]
            if args.ignoreminmax:
                x = np.random.normal(l[0], l[1])
            else:
                x = min(max(np.random.normal(l[0], l[1]), l[2]), l[3])
            txt = txt.replace(original[match.start():match.end()], str(x), 1)

        with open(output_name.format(i), 'w') as f:
            f.write(txt)

