import numpy as np
import json, sys, os


def read_json(path):
    with open(path, 'r') as f:
        data = json.load(f)
    return data



def read_param(data):
    """
    To complete loading the DS parameters: A and b
    """
    
    K = data['K']
    M = data['M']
    Priors = np.array(data['Priors'])
    Mu = np.array(data['Mu']).reshape(K, -1)
    Sigma = np.array(data['Sigma']).reshape(K, M, M)

    att = np.array(data['attractor'])

    A = np.array(data['A']).reshape(K, M, M)
    b = np.array(data['b']).reshape(M, K)

    for k in range(K):
                A[k, :, :] = A[k, :, :].T
                # b[k] = b[k].T


    
    return K, M, Priors, Mu, Sigma, att, A, b



def knn_search(Mu, att, size):
    distances = np.zeros(len(Mu))
    index = 0
    for mu in Mu:
        distances[index] = np.linalg.norm(mu - att)
        index = index + 1

    order = []
    for i in np.arange(0, len(Mu)):
        cur_value = distances[i]
        cur_index = 0
        for j in np.arange(0, len(Mu)):
            if distances[j] < cur_value:
                cur_index += 1
        order.append(cur_index)

    return order


class ds_gmms:
    Mu = None
    Sigma = None
    Priors = None


def adjust_covariances(Priors, Sigma, tot_scale_fact, rel_scale_fact):
    # Check Relative Covariance Matrix Eigenvalues
    est_K = len(Sigma)
    dim = np.shape(Sigma)[1]
    Vs = np.zeros((est_K, dim, dim))
    Ls = np.zeros((est_K, dim, dim))
    p1_eig = []
    p2_eig = []
    p3_eig = []

    baseline_prior = (0.5 / len(Priors))

    for k in np.arange(est_K):
        w, v = np.linalg.eig(Sigma[k])
        Ls[k] = np.diag(w)
        Vs[k] = v.copy()
        if not all(sorted(w) == w):
            ids = w.argsort()
            L_ = np.sort(w)
            Ls[k] = np.diag(L_)
            Vs[k] = Vs[k][:, ids]

        if Priors[k] > baseline_prior:
            Ls[k] = tot_scale_fact * Ls[k]

        # 提取最大的两个特征值
        lambda_1 = Ls[k][0][0]
        lambda_2 = Ls[k][1][1]
        p1_eig.append(lambda_1)
        p2_eig.append(lambda_2)
        if dim == 3:
            lambda_3 = Ls[k][2][2]
            p3_eig.append(lambda_3)
        Sigma[k] = Vs[k] @ Ls[k] @ Vs[k].T

    p1_eig = np.array(p1_eig)
    p2_eig = np.array(p2_eig)
    p3_eig = np.array(p3_eig)

    if dim == 2:
        cov_ratios = np.array(p1_eig / p2_eig)
        for k in np.arange(0, est_K):
            if cov_ratios[k] < rel_scale_fact:
                lambda_1 = p1_eig[k]
                lambda_2 = p2_eig[k]
                lambda_1_ = lambda_1 + lambda_2 * (rel_scale_fact - cov_ratios[k])
                Sigma[k] = Vs[k] @ np.diag([lambda_1_, lambda_2]) @ Vs[k].T
    elif dim == 3:
        cov_ratios = np.array(p2_eig / p3_eig)
        for k in np.arange(0, est_K):
            if cov_ratios[k] < rel_scale_fact:
                lambda_1 = p1_eig[k]
                lambda_2 = p2_eig[k]
                lambda_3 = p3_eig[k]
                lambda_2_ = lambda_2 + lambda_3 * (rel_scale_fact - cov_ratios[k])
                lambda_1_ = lambda_1 + lambda_3 * (rel_scale_fact - cov_ratios[k])
                Sigma[k] = Vs[k] @ np.diag([lambda_1_, lambda_2_, lambda_3]) @ Vs[k].T

    return Sigma



def rearrange_clusters(Priors, Mu, Sigma, att):
    Mu = Mu.T
    dim = len(Mu)
    # rearrange the probability arrangement
    idx = knn_search(Mu.T, att.reshape(len(att)), len(Mu[0]))
    Priors_old = Priors.copy()
    Mu_old = Mu.copy()
    Sigma_old = Sigma.copy()
    for i in np.arange(len(idx)):
        Priors[idx[i]] = Priors_old[i]
        Mu[:, idx[i]] = Mu_old[:, i]
        Sigma[idx[i]] = Sigma_old[i]
    # Make the closest Gaussian isotropic and place it at the attractor location
    Sigma[0] = 1 * np.max(np.diag(Sigma[0])) * np.eye(dim)
    Mu[:, 0] = att.reshape(len(att))
    # gmm = GMM(len(Mu[0]), Priors, Mu.T, Sigma)  # checked 10/22/2022

    # This is recommended to get smoother streamlines/global dynamics
    # This is used to expand the covariance
    ds_gmm = ds_gmms()
    ds_gmm.Mu = Mu
    ds_gmm.Sigma = Sigma
    ds_gmm.Priors = Priors
    adjusts_C = 1
    if adjusts_C == 1:
        if dim == 2:
            tot_dilation_factor = 1
            rel_dilation_fact = 0.25
        else:
            # this is for dim == 3
            tot_dilation_factor = 1
            rel_dilation_fact = 0.75
        Sigma_ = adjust_covariances(ds_gmm.Priors, ds_gmm.Sigma, tot_dilation_factor, rel_dilation_fact)
        ds_gmm.Sigma = Sigma_

    return ds_gmm



def my_gaussPDF(X, Mu, Sigma):
    """
    %MY_GAUSSPDF computes the Probability Density Function (PDF) of a
    % multivariate Gaussian represented by a mean and covariance matrix.
    %
    % Inputs -----------------------------------------------------------------
    %       o X     : (N x M), a data set with M samples each being of dimension N.
    %                          each column corresponds to a datapoint
    %       o Mu    : (N x 1), an Nx1 vector corresponding to the mean of the
    %							Gaussian function
    %       o Sigma : (N x N), an NxN matrix representing the covariance matrix
    %						   of the Gaussian function
    % Outputs ----------------------------------------------------------------
    %       o prob  : (1 x M),  a 1xM vector representing the probabilities for each
    %                           M datapoints given Mu and Sigma
    %%
    This function has passed the check ( 08/11/22 )
    """
    # Auxiliary Variables
    N = len(X)
    M = len(X[0])

    # Output Variable
    prob = np.zeros(M)

    # Demean Data
    X = X - Mu

    # Compute Probabilities
    A = X.T @ np.linalg.inv(Sigma)
    B = A * X.T
    # 1) The exponential term is the inner products of the zero-mean data
    exp_term = np.sum(B, axis=1)

    # 2) Compute Equation (2) there is a real-min here but I did'nt add it
    prob = np.exp(-0.5 * exp_term) / np.sqrt((2*np.pi)**N * (np.abs(np.linalg.det(Sigma))))

    return prob

def posterior_probs_gmm(x, gmm, type):
    N = len(x)
    M = len(x[0])

    # Unpack gmm
    Mu = gmm.Mu
    Priors = gmm.Priors
    Sigma = gmm.Sigma
    K = len(Priors)
    # Compute mixing weights for multiple dynamics
    Px_k = np.zeros((K, M))

    # Compute probabilities p(x^i|k)
    for k in np.arange(K):
        Px_k[k, :] = my_gaussPDF(x, Mu[:, k].reshape(N, 1), Sigma[k, :, :])

    # Compute posterior probabilities p(k|x) -- FAST WAY --- %%%
    alpha_Px_k = np.repeat(Priors.reshape(len(Priors),1), M, axis=1) * Px_k

    if type == 'norm':
        Pk_x = alpha_Px_k / np.repeat(np.sum(alpha_Px_k, axis=0, keepdims=True), K, axis=0)
    else:
        Pk_x = alpha_Px_k

    return Pk_x


def lpv_ds(x, ds_gmm, A_g, b_g):
    N, M = x.shape  # N for dim and M for data point number
    K = len(ds_gmm.Priors)

    # Posterior Probabilities per local DS
    beta_k_x = posterior_probs_gmm(x, ds_gmm, 'norm')  # K*M matrix

    x_dot = np.zeros((N, M))
    for i in np.arange(M):
        f_g = np.zeros((N, K))  # dim * cluster p（k）* （A_k @ x + b_k)
        if b_g.shape[1] > 1:
            for k in np.arange(K):
                f_g[:, k] = beta_k_x[k][i] * (A_g[k] @ x[:, i] + b_g[:, k])
            f_g = np.sum(f_g, axis=1)

        else:
            # Estimate the Global Dynamics component as Linear DS
            f_g = (A_g @ x[:, i] + b_g.reshape(-1))

        x_dot[:, i] = f_g

    return x_dot


class lpv_ds_class:

    def __init__(self):

        js_path = 'output.json'
        original_js = read_json(js_path)
        self.K, self.M, self.Priors, self.Mu, self.Sigma, self.att, self.A, self.b= read_param(original_js)


        self.ds_struct = rearrange_clusters(self.Priors, self.Mu, self.Sigma, self.att)
        

    def step(self, x):
        ds_handle = lambda x_velo: lpv_ds(x_velo, self.ds_struct, self.A, self.b)
        
        x = np.array(x).reshape(-1, 1)
        xd = ds_handle(x)

        return xd



if __name__ == "__main__":
    a = lpv_ds_class()

    a.step(np.array([0, 0, 0]).reshape(-1, 1), dt =1 )













