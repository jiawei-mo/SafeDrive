#include "parameters.hpp"
class GSVFetcher
{
public:
	GSVFetcher();
	GSVFetcher(string _key);
	Mat get(Size, float lan, float lon, float head, float pitch);
private:
	string key;
};
