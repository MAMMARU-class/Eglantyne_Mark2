#include <vector>

class CubicSpline1D {
public:
    std::vector<double> a,b,c,d,x;

    void build(const std::vector<double>& x_,
               const std::vector<double>& y_)
    {
        x = x_;
        int n = x.size() - 1;

        std::vector<double> h(n);
        for(int i=0;i<n;i++)
            h[i] = x[i+1] - x[i];

        std::vector<double> alpha(n);
        for(int i=1;i<n;i++)
            alpha[i] =
                (3.0/h[i])*(y_[i+1]-y_[i])
              - (3.0/h[i-1])*(y_[i]-y_[i-1]);

        std::vector<double> l(n+1), mu(n+1), z(n+1);
        a = y_;
        b.resize(n);
        c.resize(n+1);
        d.resize(n);

        l[0]=1; mu[0]=0; z[0]=0;

        for(int i=1;i<n;i++){
            l[i]=2*(x[i+1]-x[i-1]) - h[i-1]*mu[i-1];
            mu[i]=h[i]/l[i];
            z[i]=(alpha[i]-h[i-1]*z[i-1])/l[i];
        }

        l[n]=1; z[n]=0; c[n]=0;

        for(int j=n-1;j>=0;j--){
            c[j]=z[j]-mu[j]*c[j+1];
            b[j]=(a[j+1]-a[j])/h[j]
                 - h[j]*(c[j+1]+2*c[j])/3.0;
            d[j]=(c[j+1]-c[j])/(3.0*h[j]);
        }
    }

    double eval(double xp) const
    {
        int n = x.size() - 1;
        int i = n-1;

        for(int j=0;j<n;j++){
            if(xp >= x[j] && xp <= x[j+1]){
                i=j; break;
            }
        }

        double dx = xp - x[i];
        return a[i]
             + b[i]*dx
             + c[i]*dx*dx
             + d[i]*dx*dx*dx;
    }
};
