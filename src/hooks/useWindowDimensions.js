import { useState, useEffect } from "react";

// Function to get the current window dimensions and aspect ratio
function getWindowDimensions() {
  const { innerWidth: width, innerHeight: height } = window;
  return {
    width,
    height,
    aspectRatio: height / width,
  };
}

// Custom hook to track and update window dimensions
export default function useWindowDimensions() {
  // State to hold the current window dimensions
  const [windowDimensions, setWindowDimensions] = useState(getWindowDimensions());

  // Effect to update dimensions on window resize
  useEffect(() => {
    // Handler for window resize
    function handleResize() {
      setWindowDimensions(getWindowDimensions());
    }

    // Add resize event listener
    window.addEventListener("resize", handleResize);

    // Cleanup event listener on component unmount
    return () => window.removeEventListener("resize", handleResize);
  }, []);

  return windowDimensions;
}
