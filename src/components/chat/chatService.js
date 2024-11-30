import axios from 'axios';

const API_BASE = 'http://localhost:5000/api';

export const sendChatMessage = async (message) => {
    const response = await axios.post(`${API_BASE}/chat`, { message });
    return response.data;
};
