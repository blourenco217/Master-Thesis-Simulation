
MPC_OPTS = {
    'ipopt': {
        'max_iter': 2000,
        'print_level': 0,
        'mu_init': 0.01,
        'tol': 1e-8,
        'warm_start_init_point': 'yes',
        'warm_start_bound_push': 1e-9,
        'warm_start_bound_frac': 1e-9,
        'warm_start_slack_bound_frac': 1e-9,
        'warm_start_slack_bound_push': 1e-9,
        'warm_start_mult_bound_push': 1e-9,
        'mu_strategy': 'adaptive',
    },
    'print_time': 0,
    'verbose' : False,
    'expand' : True
}

def rk4(f, x, u, h):
    "Runge-Kutta 4th Order Method for discretazation"
    k1 = f(x, u)
    k2 = f(x + h/2*k1, u)
    k3 = f(x + h/2*k2, u)
    k4 = f(x + h * k3, u)
    return x + (h / 6) * (k1 + 2 * k2 + 2 * k3 + k4)