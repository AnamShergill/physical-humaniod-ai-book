import React from 'react';
import styles from './CodeBlock.module.css';

// Component for displaying code examples with explanations
function CodeBlock({title, language, code, explanation, relatedConcepts = []}) {
  return (
    <div className="textbook-code-block">
      <div className="code-header">
        <h4>{title}</h4>
        <span className={`code-language language-${language}`}>{language}</span>
      </div>
      <pre><code className={`language-${language}`}>{code}</code></pre>
      {explanation && (
        <div className="code-explanation">
          <p>{explanation}</p>
        </div>
      )}
      {relatedConcepts.length > 0 && (
        <div className="code-related-concepts">
          <strong>Related Concepts:</strong> {relatedConcepts.join(', ')}
        </div>
      )}
    </div>
  );
}

export default CodeBlock;