import axios from 'axios';

const API_BASE = 'http://localhost:5000/api';

export const fetchPlannedActions = async () => {
    const response = await axios.get(`${API_BASE}/queue`);
    return response.data;
};
