import tensorflow as tf
from tensorflow.keras.layers import Layer, Input, LSTM
from tensorflow.keras.models import Model


class InputVecLayer(Layer):
    """
    Custom input processing layer for ECPIDUNN.
    
    Depending on the mode, it either:
    - Concatenates the error vector with parameters for NN input.
    - Updates the internal state for feedback-based PID adjustment.
    """
    def __init__(self, error_vec, pt, **kwargs):
        super(InputVecLayer, self).__init__(**kwargs)
        self.error_vec = error_vec  # Tensor storing current error and feedback info
        self.pt = pt                # Tensor storing parameter vector

    def call(self, error, hidden_state=None, control_signal=None, mode="feedback"):
        """
        Process the error and optionally hidden state and control signal.

        Parameters:
            error: Current error value
            hidden_state: Previous hidden state from NN
            control_signal: Last control signal
            mode: 'concat' for NN input, 'feedback' for PID update

        Returns:
            Tensor representing either concatenated NN input or updated error vector
        """
        if mode == "concat":
            # Update error vector with current error
            self.error_vec[0].assign(error)
            # Concatenate error vector with parameter vector and expand dimensions for LSTM input
            return tf.expand_dims(tf.expand_dims(tf.concat([self.error_vec, self.pt], axis=-1), axis=0), axis=0)

        if mode == "feedback":
            # Update parameter tensor with hidden state
            self.pt.assign(tf.squeeze(hidden_state, axis=0))
            # Update error vector with current error, control signal, and difference
            self.error_vec.assign([error, control_signal, error - control_signal])
            return self.error_vec


class ECPIDUNN:
    """
    Error-Centric PID Untrained Neural Network (ECPIDUNN).

    Combines classical PID control with a feedforward neural network to dynamically adjust
    PID coefficients based on error feedback.

    Attributes:
        kp, ki, kd: Initial PID coefficients
        pt: Parameter vector (size must be 3)
        dt: Time step for PID integration
        tau: Stabilization factor for improved PID
        nn_units: List defining units in each LSTM hidden layer
    """
    def __init__(self, kp, ki, kd, pt, dt=0.01, tau=1.0, nn_units=[20, 16, 3], **kwargs):
        super(ECPIDUNN, self).__init__(**kwargs)
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.tau = tau
        self.dt = dt

        # Initialize PID internal state
        self.control_signal = 0.0
        self.last_error = 0.0
        self.integral = 0.0

        # Validate and store parameter vector
        assert len(pt) == 3, "Parameter vector 'pt' must have exactly 3 elements."
        self.pt = tf.Variable(pt, dtype=tf.float32)
        self.error_vec = tf.Variable([0.0, 0.0, 0.0], dtype=tf.float32)

        # Initialize input and hidden layers
        self.input_layer = InputVecLayer(self.error_vec, self.pt)
        self.nn_layers = self.build_nn(nn_units)

    def build_nn(self, nn_units):
        """
        Construct feedforward LSTM layers for the network.

        Parameters:
            nn_units: List of integers specifying number of units per layer

        Returns:
            List of LSTM layers
        """
        nn_layers = []
        for units in nn_units:
            nn_layers.append(LSTM(units, activation='relu', return_sequences=True))
        return nn_layers

    def build_controller(self):
        """
        Build the complete ECPIDUNN model combining input, hidden layers, and PID.

        Returns:
            Keras Model
        """
        error_input = Input(shape=(1,))
        output = self.call(error_input)
        return Model(inputs=error_input, outputs=output)

    def pass_through_nn(self, inputs):
        """
        Sequentially pass the input tensor through all LSTM hidden layers.

        Parameters:
            inputs: Tensor input

        Returns:
            Tensor output from the last hidden layer
        """
        x = inputs
        for nn_layer in self.nn_layers:
            x = nn_layer(x)
        return x

    def dynamic_compute(self, hidden_state, error):
        """
        Compute the PID control signal with dynamic coefficient adjustment.

        PID coefficients are modified based on NN hidden state and current error.

        Parameters:
            hidden_state: Output from NN representing dynamic adjustment factors
            error: Current error

        Returns:
            Updated control signal
        """
        # Update integral term
        self.integral += error * self.dt

        # Dynamically adjust PID coefficients based on NN outputs
        self.kp += hidden_state[:, :, 0] * (error - self.control_signal / self.dt)
        self.ki += hidden_state[:, :, 1] * (error - self.control_signal / self.dt)
        self.kd += hidden_state[:, :, 2] * (error - self.control_signal / self.dt)

        # PID computation
        P = self.kp * error
        I = self.ki * self.integral / self.tau
        D = self.kd * self.tau * (error - self.last_error) / self.dt

        # Update internal states
        self.last_error = error
        self.control_signal = P + I + D

        return self.control_signal

    def call(self, error):
        """
        Forward pass through ECPIDUNN.

        Steps:
            1. Update input layer with error (concat mode for NN input)
            2. Pass through LSTM layers
            3. Compute dynamic PID control signal
            4. Update error vector in feedback mode

        Parameters:
            error: Tensor containing current error

        Returns:
            Control signal as tensor
        """
        # Prepare NN input
        concatenated = self.input_layer(error[0, 0], mode="concat")
        # Get dynamic adjustment factors from NN
        hidden_state = self.pass_through_nn(concatenated)
        # Compute updated PID control signal
        control_signal = self.dynamic_compute(hidden_state, error[0, 0])
        # Update error vector for feedback
        self.error_vec = self.input_layer(error[0, 0], hidden_state[0], control_signal[0, 0], mode="feedback")
        return control_signal

    def get_weights(self):
        """
        Retrieve all LSTM layer weights.

        Returns:
            Dictionary mapping layer names to weight tensors
        """
        weights = {}
        for idx, nn_layer in enumerate(self.nn_layers):
            weights[f'layer_{idx + 1}'] = nn_layer.get_weights()
        return weights

