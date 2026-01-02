import React from 'react';
import clsx from 'clsx';
import styles from './styles.module.css';

// Enhanced Typography component with new visual design
export default function Typography({ children, variant = 'body', className = '', ...props }) {
  const baseClass = styles.typography;
  const variantClass = styles[`typography--${variant}`];
  const combinedClass = clsx(baseClass, variantClass, className);

  const Element = variant.startsWith('h') ? variant : 'div';

  return (
    <Element className={combinedClass} {...props}>
      {children}
    </Element>
  );
}