import React from 'react';
import OriginalLayout from '@theme-original/Layout';
import FloatingAssistant from '@site/src/components/Chatbot/FloatingAssistant';

// Enhanced layout with new visual design
export default function Layout(props) {
  return (
    <>
      <OriginalLayout {...props} />
      <FloatingAssistant />
    </>
  );
}