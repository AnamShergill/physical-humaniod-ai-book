import React from 'react';
import MDXComponents from '@theme-original/MDXComponents';
import CodeBlock from '@theme/CodeBlock';

// Enhanced MDX components with new visual design
const customComponents = {
  // Enhanced CodeBlock with new styling
  CodeBlock: (props) => (
    <div className="code-block-wrapper">
      <CodeBlock {...props} />
    </div>
  ),
};

export default {
  ...MDXComponents,
  ...customComponents,
};