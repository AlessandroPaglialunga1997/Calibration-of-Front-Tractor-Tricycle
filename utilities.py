def compare_matrices(A, B):
    if (A.shape != B.shape):
        return False
    else:
        for i in range(0, A.shape[0]):
            for j in range(0, B.shape[1]):
                if(A[i][j] != B[i][j]):
                    return False
        return True