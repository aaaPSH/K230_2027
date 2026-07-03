"""Pure Python linear Kalman filter utility."""


class KalmanFilter:
    """
    Linear Kalman filter for small state vectors.

    Model:
        x = A x + B u
        z = H x

    All vectors and matrices use plain Python lists, so this class can run in
    embedded Python environments without NumPy.
    """

    def __init__(
        self,
        state,
        covariance,
        transition_matrix,
        measurement_matrix,
        process_noise,
        measurement_noise,
        control_matrix=None,
    ):
        self.x = _as_vector(state)
        self.P = _copy_matrix(covariance)
        self.A = _copy_matrix(transition_matrix)
        self.H = _copy_matrix(measurement_matrix)
        self.Q = _copy_matrix(process_noise)
        self.R = _copy_matrix(measurement_noise)
        self.B = None if control_matrix is None else _copy_matrix(control_matrix)

        self.state_size = len(self.x)
        self.measurement_size = len(self.H)
        self._validate_model()

    def reset(self, state, covariance=None):
        self.x = _as_vector(state)
        self.state_size = len(self.x)
        if covariance is not None:
            self.P = _copy_matrix(covariance)
        self._validate_model()

    def set_transition_matrix(self, transition_matrix):
        self.A = _copy_matrix(transition_matrix)
        _require_shape(self.A, self.state_size, self.state_size, "transition_matrix")

    def set_process_noise(self, process_noise):
        self.Q = _copy_matrix(process_noise)
        _require_square(self.Q, self.state_size, "process_noise")

    def set_measurement_noise(self, measurement_noise):
        self.R = _copy_matrix(measurement_noise)
        _require_square(self.R, self.measurement_size, "measurement_noise")

    def predict(self, control=None, transition_matrix=None, process_noise=None):
        """
        Predict next state:
            x = A x + B u
            P = A P A^T + Q
        """
        A = self.A if transition_matrix is None else _copy_matrix(transition_matrix)
        Q = self.Q if process_noise is None else _copy_matrix(process_noise)
        _require_shape(A, self.state_size, self.state_size, "transition_matrix")
        _require_square(Q, self.state_size, "process_noise")

        self.x = _mat_vec_mul(A, self.x)
        if control is not None:
            if self.B is None:
                raise ValueError("control_matrix is required when control is used")
            u = _as_vector(control)
            self.x = _vec_add(self.x, _mat_vec_mul(self.B, u))

        self.P = _mat_add(_mat_mul(_mat_mul(A, self.P), _transpose(A)), Q)
        return self.x[:]

    def update(self, measurement, measurement_matrix=None, measurement_noise=None):
        """
        Correct state with measurement:
            y = z - H x
            K = P H^T (H P H^T + R)^-1
            x = x + K y
        """
        z = _as_vector(measurement)
        H = self.H if measurement_matrix is None else _copy_matrix(measurement_matrix)
        R = self.R if measurement_noise is None else _copy_matrix(measurement_noise)

        measurement_size = len(z)
        _require_shape(H, measurement_size, self.state_size, "measurement_matrix")
        _require_square(R, measurement_size, "measurement_noise")

        z_pred = _mat_vec_mul(H, self.x)
        residual = _vec_sub(z, z_pred)
        H_t = _transpose(H)
        innovation_cov = _mat_add(_mat_mul(_mat_mul(H, self.P), H_t), R)
        kalman_gain = _mat_mul(
            _mat_mul(self.P, H_t),
            _inverse(innovation_cov),
        )

        self.x = _vec_add(self.x, _mat_vec_mul(kalman_gain, residual))

        # Joseph form keeps covariance symmetric and positive better in practice.
        identity = _identity(self.state_size)
        gain_h = _mat_mul(kalman_gain, H)
        identity_minus_gain_h = _mat_sub(identity, gain_h)
        self.P = _mat_add(
            _mat_mul(
                _mat_mul(identity_minus_gain_h, self.P),
                _transpose(identity_minus_gain_h),
            ),
            _mat_mul(_mat_mul(kalman_gain, R), _transpose(kalman_gain)),
        )
        return self.x[:]

    def step(
        self,
        measurement,
        control=None,
        transition_matrix=None,
        process_noise=None,
        measurement_matrix=None,
        measurement_noise=None,
    ):
        self.predict(
            control=control,
            transition_matrix=transition_matrix,
            process_noise=process_noise,
        )
        return self.update(
            measurement,
            measurement_matrix=measurement_matrix,
            measurement_noise=measurement_noise,
        )

    def state(self):
        return self.x[:]

    def covariance(self):
        return _copy_matrix(self.P)

    def _validate_model(self):
        _require_square(self.P, self.state_size, "covariance")
        _require_shape(self.A, self.state_size, self.state_size, "transition_matrix")
        _require_shape(self.H, self.measurement_size, self.state_size, "measurement_matrix")
        _require_square(self.Q, self.state_size, "process_noise")
        _require_square(self.R, self.measurement_size, "measurement_noise")
        if self.B is not None:
            _require_shape(self.B, self.state_size, len(self.B[0]), "control_matrix")


def make_constant_velocity_filter(
    position,
    velocity=0.0,
    dt=1.0,
    position_variance=100.0,
    velocity_variance=100.0,
    process_variance=0.01,
    measurement_variance=1.0,
):
    """Create a 1D constant-velocity Kalman filter: state = [position, velocity]."""
    return KalmanFilter(
        state=[position, velocity],
        covariance=[
            [position_variance, 0.0],
            [0.0, velocity_variance],
        ],
        transition_matrix=[
            [1.0, dt],
            [0.0, 1.0],
        ],
        measurement_matrix=[[1.0, 0.0]],
        process_noise=[
            [process_variance, 0.0],
            [0.0, process_variance],
        ],
        measurement_noise=[[measurement_variance]],
    )


def _as_vector(values):
    return [float(value) for value in values]


def _copy_matrix(matrix):
    return [[float(value) for value in row] for row in matrix]


def _zeros(rows, cols):
    return [[0.0 for _ in range(cols)] for _ in range(rows)]


def _identity(size):
    matrix = _zeros(size, size)
    for index in range(size):
        matrix[index][index] = 1.0
    return matrix


def _transpose(matrix):
    rows = len(matrix)
    cols = len(matrix[0]) if rows else 0
    return [[matrix[row][col] for row in range(rows)] for col in range(cols)]


def _vec_add(left, right):
    _require_vector_same_size(left, right)
    return [left[index] + right[index] for index in range(len(left))]


def _vec_sub(left, right):
    _require_vector_same_size(left, right)
    return [left[index] - right[index] for index in range(len(left))]


def _mat_add(left, right):
    _require_same_shape(left, right)
    rows = len(left)
    cols = len(left[0]) if rows else 0
    return [
        [left[row][col] + right[row][col] for col in range(cols)]
        for row in range(rows)
    ]


def _mat_sub(left, right):
    _require_same_shape(left, right)
    rows = len(left)
    cols = len(left[0]) if rows else 0
    return [
        [left[row][col] - right[row][col] for col in range(cols)]
        for row in range(rows)
    ]


def _mat_mul(left, right):
    left_rows = len(left)
    left_cols = len(left[0]) if left_rows else 0
    right_rows = len(right)
    right_cols = len(right[0]) if right_rows else 0
    if left_cols != right_rows:
        raise ValueError("matrix multiply shape mismatch")

    result = _zeros(left_rows, right_cols)
    for row in range(left_rows):
        for col in range(right_cols):
            total = 0.0
            for inner in range(left_cols):
                total += left[row][inner] * right[inner][col]
            result[row][col] = total
    return result


def _mat_vec_mul(matrix, vector):
    rows = len(matrix)
    cols = len(matrix[0]) if rows else 0
    if cols != len(vector):
        raise ValueError("matrix/vector multiply shape mismatch")

    result = [0.0 for _ in range(rows)]
    for row in range(rows):
        total = 0.0
        for col in range(cols):
            total += matrix[row][col] * vector[col]
        result[row] = total
    return result


def _inverse(matrix, eps=1e-12):
    size = len(matrix)
    _require_square(matrix, size, "matrix")
    work = _copy_matrix(matrix)
    inv = _identity(size)

    for col in range(size):
        pivot_row = col
        pivot_abs = abs(work[col][col])
        for row in range(col + 1, size):
            value_abs = abs(work[row][col])
            if value_abs > pivot_abs:
                pivot_abs = value_abs
                pivot_row = row

        if pivot_abs < eps:
            raise ValueError("matrix is singular or ill-conditioned")

        if pivot_row != col:
            work[col], work[pivot_row] = work[pivot_row], work[col]
            inv[col], inv[pivot_row] = inv[pivot_row], inv[col]

        pivot = work[col][col]
        for item in range(size):
            work[col][item] /= pivot
            inv[col][item] /= pivot

        for row in range(size):
            if row == col:
                continue
            factor = work[row][col]
            if factor == 0.0:
                continue
            for item in range(size):
                work[row][item] -= factor * work[col][item]
                inv[row][item] -= factor * inv[col][item]

    return inv


def _require_vector_same_size(left, right):
    if len(left) != len(right):
        raise ValueError("vector size mismatch")


def _require_same_shape(left, right):
    if len(left) != len(right):
        raise ValueError("matrix row size mismatch")
    for row in range(len(left)):
        if len(left[row]) != len(right[row]):
            raise ValueError("matrix column size mismatch")


def _require_square(matrix, size, name):
    _require_shape(matrix, size, size, name)


def _require_shape(matrix, rows, cols, name):
    if len(matrix) != rows:
        raise ValueError(name + " row size mismatch")
    for row in matrix:
        if len(row) != cols:
            raise ValueError(name + " column size mismatch")
