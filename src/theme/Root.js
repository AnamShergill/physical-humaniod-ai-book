import React from 'react';
import {Provider} from 'react-wrap-balancer';

// Enhanced theme wrapper with new visual design
export default function Root({children}) {
  return <Provider>{children}</Provider>;
}