#include <iostream>
#include <fstream>
#include <cstring>
#include <boost/program_options.hpp>
#include <boost/tokenizer.hpp>

#include <lazycbs/mapf/instance.h>

using namespace std;
namespace mapf {

Map::Map(int _rows, int _cols)
  : rows(_rows), cols(_cols)
  , mem(new char[rows*cols]) { }

Map::~Map(void) {
  delete[] mem;
}

void expect(ifstream& f, std::string s) {
  std::string match(s.size(), '\0');
  f.read(&match[0], s.size());
  assert(match == s);
}

Map load_ecbs_map(std::string fname) {
  ifstream mapfile(fname.c_str());

  Map ml(1, 1);
  assert(0);
  return ml;
}

static const std::streamsize skip = std::numeric_limits<std::streamsize>::max();

typedef boost::char_separator<char> sep_t;
typedef boost::tokenizer< sep_t > tok_t;
Map load_movingai_map(std::string fname) {
  ifstream mapfile(fname.c_str());
  string line;

  sep_t sep(" ");
  int rows, cols;

  // Read the header
  getline(mapfile, line); // type octile
  getline(mapfile, line); // height <rows>
  {
    tok_t tok(line, sep);
    auto it = tok.begin(); ++it;
    rows = atoi( (*it).c_str() );
  }
  getline(mapfile, line); // height <rows>
  {
    tok_t tok(line, sep);
    auto it = tok.begin(); ++it;
    cols = atoi( (*it).c_str() );
  }
  getline(mapfile, line); // map

  // Add padding at either side, for historical reasons.
  Map m(rows+2, cols+2);
  char* mem(m.mem);

  memset(mem, -1, (rows+2)*(cols+2));
  mem += cols+2;
  for(int r = 0; r < rows; ++r) {
    getline(mapfile, line);

    ++mem; // Skip first col
    for(int c = 0; c < cols; ++c) {
      switch(line[c]) {
      case '.':
      case 'G':
        *mem = 0;
        break;
      default:
        break;
      }
      ++mem;
    }
    ++mem; // Skip last col
  }
  return m;
}

Agents load_ecbs_scenario(std::string fname) {
  // Loader code adapted from ECBS
  Agents al;
  string line;

  ifstream myfile (fname.c_str());
  if (myfile.is_open()) {
    getline (myfile,line);
    sep_t sep(",");
    tok_t tok(line, sep);
    tok_t::iterator beg=tok.begin();
    int count = atoi ( (*beg).c_str() );
    for (int i=0; i<count; i++) {
      getline (myfile, line);
      tok_t col_tok(line, sep);
      tok_t::iterator c_beg=col_tok.begin();
      pair<int,int> curr_pair;
      int rs = atoi ( (*c_beg).c_str() ); c_beg++;
      int cs = atoi ( (*c_beg).c_str() ); c_beg++;
      int re = atoi ( (*c_beg).c_str() ); c_beg++;
      int ce = atoi ( (*c_beg).c_str() ); 
      al.push(Agents::agent(rs, cs, re, ce));
    }
    myfile.close();
  }
  else
    cerr << "Agents file not found." << std::endl;

  return al;
}

Agents load_movingai_scenario(std::string fname, int upto) {
  Agents al;

  string line;
  ifstream myfile(fname.c_str());
  
  if (myfile.is_open()) {
    boost::char_separator<char> sep("\t");

    std::getline(myfile,line); // Ditch the first line
    for (int i=0; i < upto; i++) {
      std::getline(myfile, line);
      boost::tokenizer< boost::char_separator<char> > col_tok(line, sep);
      boost::tokenizer< boost::char_separator<char> >::iterator c_beg=col_tok.begin();
      // EX: 64	brc202d.map	530	481	446	403	444	182	259.12489166
      ++c_beg; // Length
      ++c_beg; // Map
      ++c_beg; // Rows
      ++c_beg; // Cols
      
      // The maps have been padded, so 'real' cells are still indexed from one.
      int c_s = atoi((*c_beg).c_str())+1; ++c_beg;
      int r_s = atoi((*c_beg).c_str())+1; ++c_beg;
      int c_e = atoi((*c_beg).c_str())+1; ++c_beg;
      int r_e = atoi((*c_beg).c_str())+1; ++c_beg;

      al.push(Agents::agent(r_s, c_s, r_e, c_e));
    }
    myfile.close();
  }
  else
    cerr << "Agents file not found." << std::endl;
  return al;
}

};
