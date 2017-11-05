#include <iostream>
#include <iomanip>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <random>

#include "gnuplot_i.hpp"

using namespace std;

double int_mc(double(*)(double), double&, double, double, int,
std::default_random_engine &generator, std::uniform_real_distribution<double>& distribution);
double f(double);
double fva(double);
void wait_for_key ();

int main()
{
    Gnuplot g("lines");

    double a = 0.0;              // left endpoint
    double b = 1.0;               // right endpoint
    int N = 500;

    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution(a,b);

    double varest, mc, va, vn;
    std::vector<double> x, err, var_e, var_t;

    for (int n=5; n <= N; n++)
    {
        mc = int_mc(f, varest, a, b, n, generator, distribution);
        va = (1.0/n)*(16.0/9.0);
        double tmp=0.0;
        double dice;
        for(int i=1; i<n; i++){
          dice = distribution(generator);
          tmp += (pow(f(dice),2)-pow(mc,2));
        }
        tmp = (1.0/n)*tmp;
        vn = (1.0/(n-1))*tmp;
        x.push_back(n);
        err.push_back(fabs(mc-1.0));
        var_e.push_back(fabs(va));
        var_t.push_back(fabs(vn));
    }

    g.reset_plot();
    g.set_grid();
    g.set_style("lines").plot_xy(x,err,"erreur");
    g.set_style("lines").plot_xy(x,var_e,"var analytique");
    g.set_style("lines").plot_xy(x,var_t,"var numerique");

    wait_for_key();

    return 0;
}

/*
    Fonction à intégrer
*/
    double f(double x)
{
    double y = 5*x*x*x*x;
    return y;
}


/*
    Integration de Monte-Carlo
*/
double int_mc(double(*f)(double), double& varest, double a, double b, int n,
  std::default_random_engine &generator, std::uniform_real_distribution<double>& distribution)
{
    double res=0.0;
    double dice_roll;
    for(int i=1; i<n; i++){
      dice_roll = distribution(generator);
      res += f(dice_roll);
    }
    return res/(1.0*n);
}


void wait_for_key ()
{
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__TOS_WIN__)  // every keypress registered, also arrow keys
    cout << endl << "Press any key to continue..." << endl;

    FlushConsoleInputBuffer(GetStdHandle(STD_INPUT_HANDLE));
    _getch();
#elif defined(unix) || defined(__unix) || defined(__unix__) || defined(__APPLE__)
    cout << endl << "Press ENTER to continue..." << endl;

    std::cin.clear();
    std::cin.ignore(std::cin.rdbuf()->in_avail());
    std::cin.get();
#endif
    return;
}
