# simtr.py - utility to convert ignitor sim trace files to
# a graph form.  Reads in trace time and IO data and converts
# to a CSV fixed time graph format suitable for scope like graphing
import os,sys

#--------------------------
def usage():
  print('use>simtr.py infile outfile')
  sys.exit(1)


#--------------------------
def traces_to_scope(lns_in, fnameout, line_start, num_lines):
  ''' Given lines input, parse out num_lines of data at line_start
  and generate a CSV file with fixed time level output suitable for
  graphing as scope data.
  '''
  full_fnameout = fnameout + '_%d_%d.csv' % (line_start, num_lines)
  fp = open(full_fnameout, 'w')

  fp.write('time,i1,A,F,L\n') # csv file header
  fp.write('0,100,100,100,100\n') # bogus lines for chart auto-scaling
  fp.write('0,0,0,0,0\n') # bogus lines for chart auto-scaling

  lns = lns_in[line_start : line_start+num_lines]
  print('processing %d lines starting at %d line, to:%s' % (num_lines, line_start, full_fnameout))

  tm_0 = -1
  last_io_str = ''
  tm_delta = 100 # us, scope scan rate
  for l in lns:
    fi = l.find(';')
    if fi >= 0:
       l = l[:fi].strip(' ')
    for i in range(5):
      l = l.replace('  ', ' ')  
  
    ls = l.split(' ')
    if len(ls) == 2:
      tm = int(ls[0].strip())
      io = int(ls[1].strip())
      # give levels I can look at as a scope trace(0-100)
      F=64 if (io & 1) else 18
      A=60 if (io & 2) else 20
      i1=80 if (io & 0x10) else 10
      L=40 if (io & 0x80) else 32
      io_str = '%d,%d,%d,%d' % (i1,A,F,L)
      if tm_0 < 0:
        tm_0 = tm
        tm_i = tm
        last_io_str = io_str
      # write out data in fixed time CSV format, to graph
      while tm_i < tm:
        fp.write('%d,%s\n' % (tm_i,last_io_str))
        tm_i += tm_delta
      fp.write('%d,%s\n' % (tm_i, io_str))
      last_io_str = io_str     
    else:
      print('bad ln:' + str(ls))
  return full_fnameout
  
#--------------------------
def main():
  if len(sys.argv) != 3:
     usage()
   
  lns = open(sys.argv[1]).readlines() # read our trace data
  print('read %d lines trace data' % (len(lns)))
 
  fnameout = sys.argv[2] # a base filename component for output files

  # just process a sample of lines, otherwise
  # we generate too much data
  num_lines = 25
  
  # following are: <line number> <description>
  # line number is determined by viewing raw trace file to determined
  # a place I want to view, then a description of this to help with file
  # naming the various sample files.
  sample_str = '''
  1 34k
  1590 34k
  4453 24k
  5519 20k
  8431 15k
  11543 10k
  15409 34k
  '''
  samples = sample_str.split('\n') # by line
  for s2 in samples:
    s2 = s2.strip()
    if s2 == '':
       continue
    ln_i = int(s2.split(' ')[0])
    desc = s2.split(' ')[1]
    output_filename = traces_to_scope(lns, fnameout + '_' + desc, ln_i, num_lines)
    # invoke a viewer on each file
    #os.system('csvc ' + output_filename) 
#------------------
main()
