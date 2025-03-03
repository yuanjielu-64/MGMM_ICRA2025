#include <fstream>
#include <iostream>
#include <unordered_map>
#include <vector>
#include <sstream>

extern "C" int CleanResults(int argc, char **argv)
{
    const char *fnameIn = argv[1] ;
    const char *fnameOut= argv[2];
       
    std::cout
	<< "input: <" << fnameIn << ">" << std::endl
	<< "output: <" << fnameOut << ">" << std::endl;

    std:: ifstream in(fnameIn);
    std::ofstream out(fnameOut);
    std::string line;
    double val;
    int count;
    int lineno = 1;
    std::vector<double> vals;
    
    
    while(std::getline(in, line))
    {
	if(line.find("nan") != std::string::npos)
	    std::cout << "<" << line << ">" << count << " : " << lineno << std::endl;
	else
	{
	    std::istringstream inStr(line);
	    vals.clear();
	    while(inStr >> val)
		vals.push_back(val);
	    
	    if(vals.size() != 8)
		std::cout << "<" << line << ">" << count << ":" << lineno << std::endl;
	    else
	    {
		for(int i = 0; i < vals.size(); ++i)
		    out << vals[i] << " ";
		out << std::endl;	    
	    }
	}
	
	++lineno;
	
    }

    in.close();
    out.close();
    
    return 0;
}

