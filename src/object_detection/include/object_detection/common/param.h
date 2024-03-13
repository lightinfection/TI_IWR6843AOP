struct param
{
    int name, num;
    bool if_channel, if_reverse;
    double lower_limit, upper_limit, eps;
};

struct config
{
    param x, y, z, i;
};

