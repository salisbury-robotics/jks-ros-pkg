#!/usr/bin/env ruby

def insert_classname filename, classname
  text = File.read(filename)
  
  once = false
  erase = false
  File.open(filename, 'w') { |f|
    text.lines do |line|
      if line.match(/^\/\*|^\n|^#(.*)+/)
        # remove all lines following
        if(line.match(/util.c/))
          erase = true
        end
        unless erase
          f.puts line
        end
        next
      end

      unless once
        f.puts "class #{classname} {"
        f.puts "public:"
        once = true
      end
      unless erase
        f.puts line.gsub(/extern /, "")
      end
    end
    # add missing function declarations, closing paren, and endif for the header
    f.puts "double eval_objv(void);"
    f.puts "void setup_indexed_params(void);"
    f.puts "void setup_indexed_optvars(void);\n};\n#endif"
  }
end

def insert_class_scope filename, classname
  text = File.read(filename)
  text.gsub!(/^(void.*)\(|^(int.*)\(|^(double.*)\(|^(float.*)\(|^(long.*)\(/) do |m|
    m.sub!(" "," #{classname}::")
  end
  
  File.open(filename, "w") { |f|
    f.puts text
  }
  #puts text
end

def toCamelCase(string)
  string.gsub(/(^|_)(.)/) { $2.upcase }
end


unless ARGV[0]
  puts "No class name specified."
  exit 1
end

# if it's not already in camel case, we'll convert it
ARGV[0] = toCamelCase(ARGV[0])

# remove these files from the cvxgen directory, since we won't be needing them
to_delete = ["csolve.c", "csolve.m", "cvxsolve.m", "make_csolve.m", "util.c"]

directory = ARGV[1]
unless ARGV[1]
  puts "Looking under default cvxgen directory."
  directory = "cvxgen"
end

unless File.exists?(directory)
  puts "Error: please specify a valid directory."
  exit 1
end

puts "Inserting class name '#{ARGV[0]}'"

Dir[directory + "/*"].each do |f|
  if to_delete.include? File.basename(f) then
    File.delete(f)
  elsif f.include? ".c" and not f.include? "testsolver" then
    insert_class_scope f, "CVX_"+ARGV[0]
  elsif f.include? ".h" then
    #create_class
    insert_classname f, "CVX_"+ARGV[0]
  end
end

# rename the #defines for the header file to allow multiple inclusions 
# of solver.h
text = File.read("cvxgen/solver.h")
File.open("cvxgen/solver.h", "w") { |f|
  text = text.gsub(/^#ifndef SOLVER_H/, "#ifndef " + 
    "SOLVER_#{ARGV[0]}_H".upcase);
  f.puts text.gsub(/^#define SOLVER_H/, "#define " + 
    "SOLVER_#{ARGV[0]}_H".upcase);
}
  
