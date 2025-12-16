import React from 'react';
import { useLocation } from '@docusaurus/router';

export default function Breadcrumb({ items }) {
  const location = useLocation();

  return (
    <nav className="flex mb-6" aria-label="Breadcrumb">
      <ol className="inline-flex items-center space-x-1 md:space-x-3">
        <li className="inline-flex items-center">
          <a href="/" className="inline-flex items-center text-sm font-medium text-gray-700 hover:text-blue-600">
            Home
          </a>
        </li>
        {items?.map((item, index) => (
          <li key={index} className="inline-flex items-center">
            <span className="mx-2 text-gray-400">/</span>
            {index === items.length - 1 ? (
              <span className="text-sm font-medium text-gray-500">{item.label}</span>
            ) : (
              <a href={item.href} className="text-sm font-medium text-gray-700 hover:text-blue-600">
                {item.label}
              </a>
            )}
          </li>
        ))}
      </ol>
    </nav>
  );
}