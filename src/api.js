import axios from "axios";

const API_BASE_URL = "http://localhost:5000/api";

export const testBackend = async () => {
    try {
        const response = await axios.get(`${API_BASE_URL}/test`);
        return response.data;
    } catch (error) {
        console.error("Error testing backend:", error);
    }
};

export const sendCommand = async (command) => {
    try {
        const response = await axios.post(`${API_BASE_URL}/send_command`, { command });
        return response.data;
    } catch (error) {
        console.error("Error sending command:", error);
    }
};
